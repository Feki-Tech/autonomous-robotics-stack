#pragma once

#include <concepts>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace ars::core::mission {

/// Result of ticking a behavior tree node.
enum class Status : std::uint8_t { kRunning, kSuccess, kFailure };

[[nodiscard]] constexpr std::string_view to_string(Status status) {
  switch (status) {
    case Status::kRunning:
      return "running";
    case Status::kSuccess:
      return "success";
    case Status::kFailure:
      return "failure";
  }
  return "unknown";
}

/// Base class for behavior tree nodes.
///
/// A tree is ticked from the root; each tick propagates to the children a
/// composite chooses to run. `tick()` dispatches to `update()` and notifies
/// `on_halt()` overrides when a running node is preempted via `halt()`.
class Node {
 public:
  explicit Node(std::string name) : name_{std::move(name)} {}
  Node(const Node&) = delete;
  Node& operator=(const Node&) = delete;
  virtual ~Node() = default;

  [[nodiscard]] const std::string& name() const { return name_; }
  [[nodiscard]] Status status() const { return status_; }

  Status tick() {
    status_ = update();
    return status_;
  }

  /// Preempts a running node (and its subtree).
  void halt() {
    if (status_ == Status::kRunning) {
      on_halt();
      status_ = Status::kFailure;
    }
  }

 protected:
  virtual Status update() = 0;
  virtual void on_halt() {}

 private:
  std::string name_;
  Status status_{Status::kFailure};
};

using NodePtr = std::unique_ptr<Node>;

/// Leaf node running a callable until it reports success or failure.
class Action final : public Node {
 public:
  Action(std::string name, std::function<Status()> body,
         std::function<void()> halt_handler = nullptr)
      : Node{std::move(name)}, body_{std::move(body)}, halt_handler_{std::move(halt_handler)} {}

 protected:
  Status update() override { return body_(); }
  void on_halt() override {
    if (halt_handler_) {
      halt_handler_();
    }
  }

 private:
  std::function<Status()> body_;
  std::function<void()> halt_handler_;
};

/// Leaf node evaluating a predicate: success when true, failure when false.
class Condition final : public Node {
 public:
  Condition(std::string name, std::function<bool()> predicate)
      : Node{std::move(name)}, predicate_{std::move(predicate)} {}

 protected:
  Status update() override { return predicate_() ? Status::kSuccess : Status::kFailure; }

 private:
  std::function<bool()> predicate_;
};

/// Composite base: owns children and halts the one still running.
class Composite : public Node {
 public:
  Composite(std::string name, std::vector<NodePtr> children)
      : Node{std::move(name)}, children_{std::move(children)} {}

 protected:
  void on_halt() override {
    if (current_ < children_.size()) {
      children_[current_]->halt();
    }
    current_ = 0;
  }

  std::vector<NodePtr> children_;
  std::size_t current_{0};
};

/// Ticks children in order; fails on the first failure, succeeds when all
/// succeed. Memory semantics: resumes at the running child on the next tick.
class Sequence final : public Composite {
 public:
  using Composite::Composite;

 protected:
  Status update() override {
    while (current_ < children_.size()) {
      const Status status = children_[current_]->tick();
      if (status == Status::kRunning) {
        return Status::kRunning;
      }
      if (status == Status::kFailure) {
        current_ = 0;
        return Status::kFailure;
      }
      ++current_;
    }
    current_ = 0;
    return Status::kSuccess;
  }
};

/// Ticks children in order; succeeds on the first success, fails when all
/// fail. Memory semantics: resumes at the running child on the next tick.
class Selector final : public Composite {
 public:
  using Composite::Composite;

 protected:
  Status update() override {
    while (current_ < children_.size()) {
      const Status status = children_[current_]->tick();
      if (status == Status::kRunning) {
        return Status::kRunning;
      }
      if (status == Status::kSuccess) {
        current_ = 0;
        return Status::kSuccess;
      }
      ++current_;
    }
    current_ = 0;
    return Status::kFailure;
  }
};

/// Decorator base: owns a single child.
class Decorator : public Node {
 public:
  Decorator(std::string name, NodePtr child) : Node{std::move(name)}, child_{std::move(child)} {}

 protected:
  void on_halt() override { child_->halt(); }

  NodePtr child_;
};

/// Inverts the child's terminal status; running passes through.
class Inverter final : public Decorator {
 public:
  using Decorator::Decorator;

 protected:
  Status update() override {
    switch (child_->tick()) {
      case Status::kRunning:
        return Status::kRunning;
      case Status::kSuccess:
        return Status::kFailure;
      case Status::kFailure:
        return Status::kSuccess;
    }
    return Status::kFailure;
  }
};

/// Retries a failing child up to `max_attempts` times; success passes
/// through immediately. Fails once the attempt budget is exhausted.
class Retry final : public Decorator {
 public:
  Retry(std::string name, NodePtr child, std::size_t max_attempts)
      : Decorator{std::move(name), std::move(child)}, max_attempts_{max_attempts} {}

 protected:
  Status update() override {
    while (attempts_ < max_attempts_) {
      const Status status = child_->tick();
      if (status == Status::kRunning) {
        return Status::kRunning;
      }
      if (status == Status::kSuccess) {
        attempts_ = 0;
        return Status::kSuccess;
      }
      ++attempts_;
    }
    attempts_ = 0;
    return Status::kFailure;
  }

  void on_halt() override {
    Decorator::on_halt();
    attempts_ = 0;
  }

 private:
  std::size_t max_attempts_;
  std::size_t attempts_{0};
};

/// Repeats a succeeding child `iterations` times; failure passes through.
class Repeat final : public Decorator {
 public:
  Repeat(std::string name, NodePtr child, std::size_t iterations)
      : Decorator{std::move(name), std::move(child)}, iterations_{iterations} {}

 protected:
  Status update() override {
    while (completed_ < iterations_) {
      const Status status = child_->tick();
      if (status == Status::kRunning) {
        return Status::kRunning;
      }
      if (status == Status::kFailure) {
        completed_ = 0;
        return Status::kFailure;
      }
      ++completed_;
    }
    completed_ = 0;
    return Status::kSuccess;
  }

  void on_halt() override {
    Decorator::on_halt();
    completed_ = 0;
  }

 private:
  std::size_t iterations_;
  std::size_t completed_{0};
};

/// Convenience factories keeping tree construction terse and readable.
[[nodiscard]] inline NodePtr action(std::string name, std::function<Status()> body,
                                    std::function<void()> halt_handler = nullptr) {
  return std::make_unique<Action>(std::move(name), std::move(body), std::move(halt_handler));
}

[[nodiscard]] inline NodePtr condition(std::string name, std::function<bool()> predicate) {
  return std::make_unique<Condition>(std::move(name), std::move(predicate));
}

template <typename... Children>
[[nodiscard]] NodePtr sequence(std::string name, Children&&... children) {
  std::vector<NodePtr> nodes;
  nodes.reserve(sizeof...(children));
  (nodes.push_back(std::forward<Children>(children)), ...);
  return std::make_unique<Sequence>(std::move(name), std::move(nodes));
}

template <typename... Children>
[[nodiscard]] NodePtr selector(std::string name, Children&&... children) {
  std::vector<NodePtr> nodes;
  nodes.reserve(sizeof...(children));
  (nodes.push_back(std::forward<Children>(children)), ...);
  return std::make_unique<Selector>(std::move(name), std::move(nodes));
}

[[nodiscard]] inline NodePtr invert(std::string name, NodePtr child) {
  return std::make_unique<Inverter>(std::move(name), std::move(child));
}

[[nodiscard]] inline NodePtr retry(std::string name, NodePtr child, std::size_t max_attempts) {
  return std::make_unique<Retry>(std::move(name), std::move(child), max_attempts);
}

[[nodiscard]] inline NodePtr repeat(std::string name, NodePtr child, std::size_t iterations) {
  return std::make_unique<Repeat>(std::move(name), std::move(child), iterations);
}

}  // namespace ars::core::mission
