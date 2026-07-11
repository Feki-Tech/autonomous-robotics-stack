#include <gtest/gtest.h>

#include <functional>
#include <utility>
#include <vector>

#include "ars_core/mission/behavior_tree.hpp"

namespace ars::core::mission {
namespace {

/// Action stub returning a scripted series of statuses and counting ticks.
class Script {
 public:
  explicit Script(std::vector<Status> statuses) : statuses_{std::move(statuses)} {}

  Status operator()() {
    ++ticks_;
    if (next_ < statuses_.size()) {
      return statuses_[next_++];
    }
    return statuses_.back();
  }

  [[nodiscard]] int ticks() const { return ticks_; }

 private:
  std::vector<Status> statuses_;
  std::size_t next_{0};
  int ticks_{0};
};

TEST(BehaviorTreeTest, ActionForwardsBodyStatus) {
  auto ok = action("ok", [] { return Status::kSuccess; });
  auto fail = action("fail", [] { return Status::kFailure; });
  EXPECT_EQ(ok->tick(), Status::kSuccess);
  EXPECT_EQ(fail->tick(), Status::kFailure);
}

TEST(BehaviorTreeTest, ConditionMapsPredicate) {
  bool flag = true;
  auto node = condition("flag", [&flag] { return flag; });
  EXPECT_EQ(node->tick(), Status::kSuccess);
  flag = false;
  EXPECT_EQ(node->tick(), Status::kFailure);
}

TEST(BehaviorTreeTest, SequenceSucceedsWhenAllChildrenSucceed) {
  int count = 0;
  auto tree = sequence("seq",
                       action("a",
                              [&count] {
                                ++count;
                                return Status::kSuccess;
                              }),
                       action("b", [&count] {
                         ++count;
                         return Status::kSuccess;
                       }));
  EXPECT_EQ(tree->tick(), Status::kSuccess);
  EXPECT_EQ(count, 2);
}

TEST(BehaviorTreeTest, SequenceFailsFastAndSkipsRemainingChildren) {
  int reached = 0;
  auto tree = sequence("seq", action("a", [] { return Status::kFailure; }), action("b", [&reached] {
                         ++reached;
                         return Status::kSuccess;
                       }));
  EXPECT_EQ(tree->tick(), Status::kFailure);
  EXPECT_EQ(reached, 0);
}

TEST(BehaviorTreeTest, SequenceResumesAtRunningChild) {
  Script first{{Status::kSuccess}};
  Script second{{Status::kRunning, Status::kSuccess}};
  auto tree = sequence("seq", action("a", std::ref(first)), action("b", std::ref(second)));

  EXPECT_EQ(tree->tick(), Status::kRunning);
  EXPECT_EQ(tree->tick(), Status::kSuccess);
  // The first child must not be re-ticked while the second was running.
  EXPECT_EQ(first.ticks(), 1);
  EXPECT_EQ(second.ticks(), 2);
}

TEST(BehaviorTreeTest, SelectorSucceedsOnFirstSuccess) {
  int reached = 0;
  auto tree = selector("sel", action("a", [] { return Status::kFailure; }),
                       action("b", [] { return Status::kSuccess; }), action("c", [&reached] {
                         ++reached;
                         return Status::kSuccess;
                       }));
  EXPECT_EQ(tree->tick(), Status::kSuccess);
  EXPECT_EQ(reached, 0);
}

TEST(BehaviorTreeTest, SelectorFailsWhenAllChildrenFail) {
  auto tree = selector("sel", action("a", [] { return Status::kFailure; }),
                       action("b", [] { return Status::kFailure; }));
  EXPECT_EQ(tree->tick(), Status::kFailure);
}

TEST(BehaviorTreeTest, SelectorResumesAtRunningChild) {
  Script first{{Status::kFailure}};
  Script second{{Status::kRunning, Status::kSuccess}};
  auto tree = selector("sel", action("a", std::ref(first)), action("b", std::ref(second)));

  EXPECT_EQ(tree->tick(), Status::kRunning);
  EXPECT_EQ(tree->tick(), Status::kSuccess);
  EXPECT_EQ(first.ticks(), 1);
}

TEST(BehaviorTreeTest, InverterFlipsTerminalStatusOnly) {
  auto flipped = invert("not", action("ok", [] { return Status::kSuccess; }));
  EXPECT_EQ(flipped->tick(), Status::kFailure);

  auto running = invert("not", action("busy", [] { return Status::kRunning; }));
  EXPECT_EQ(running->tick(), Status::kRunning);
}

TEST(BehaviorTreeTest, RetryRecoversFromTransientFailure) {
  Script script{{Status::kFailure, Status::kFailure, Status::kSuccess}};
  auto tree = retry("retry", action("flaky", std::ref(script)), 3);
  EXPECT_EQ(tree->tick(), Status::kSuccess);
  EXPECT_EQ(script.ticks(), 3);
}

TEST(BehaviorTreeTest, RetryFailsWhenBudgetExhausted) {
  Script script{{Status::kFailure}};
  auto tree = retry("retry", action("broken", std::ref(script)), 3);
  EXPECT_EQ(tree->tick(), Status::kFailure);
  EXPECT_EQ(script.ticks(), 3);

  // Budget resets after the terminal failure.
  EXPECT_EQ(tree->tick(), Status::kFailure);
  EXPECT_EQ(script.ticks(), 6);
}

TEST(BehaviorTreeTest, RepeatRunsChildForEachIteration) {
  Script script{{Status::kSuccess}};
  auto tree = repeat("repeat", action("step", std::ref(script)), 4);
  EXPECT_EQ(tree->tick(), Status::kSuccess);
  EXPECT_EQ(script.ticks(), 4);
}

TEST(BehaviorTreeTest, RepeatFailurePassesThrough) {
  Script script{{Status::kSuccess, Status::kFailure}};
  auto tree = repeat("repeat", action("step", std::ref(script)), 4);
  EXPECT_EQ(tree->tick(), Status::kFailure);
  EXPECT_EQ(script.ticks(), 2);
}

TEST(BehaviorTreeTest, HaltNotifiesRunningActionAndResetsSequence) {
  bool halted = false;
  Script busy{{Status::kRunning}};
  Script first{{Status::kSuccess}};
  auto tree = sequence("seq", action("a", std::ref(first)),
                       action("busy", std::ref(busy), [&halted] { halted = true; }));

  EXPECT_EQ(tree->tick(), Status::kRunning);
  tree->halt();
  EXPECT_TRUE(halted);
  EXPECT_EQ(tree->status(), Status::kFailure);

  // After halt the sequence restarts from its first child.
  EXPECT_EQ(tree->tick(), Status::kRunning);
  EXPECT_EQ(first.ticks(), 2);
}

TEST(BehaviorTreeTest, HaltOnNonRunningNodeIsNoOp) {
  bool halted = false;
  auto tree = action("done", [] { return Status::kSuccess; }, [&halted] { halted = true; });
  tree->tick();
  tree->halt();
  EXPECT_FALSE(halted);
  EXPECT_EQ(tree->status(), Status::kSuccess);
}

TEST(BehaviorTreeTest, StatusToStringCoversAllValues) {
  EXPECT_EQ(to_string(Status::kRunning), "running");
  EXPECT_EQ(to_string(Status::kSuccess), "success");
  EXPECT_EQ(to_string(Status::kFailure), "failure");
}

}  // namespace
}  // namespace ars::core::mission
