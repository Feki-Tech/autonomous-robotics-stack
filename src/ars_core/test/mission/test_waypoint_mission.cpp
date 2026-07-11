#include <gtest/gtest.h>

#include <vector>

#include "ars_core/mission/waypoint_mission.hpp"

namespace ars::core::mission {
namespace {

/// Simulated navigation backend: waypoints "arrive" after a fixed number of
/// ticks, with optional scripted failures.
class FakeNavigator {
 public:
  explicit FakeNavigator(int ticks_to_arrive = 2) : ticks_to_arrive_{ticks_to_arrive} {}

  WaypointMissionCallbacks callbacks() {
    return {
        .send_goal =
            [this](geometry::Vec2<double> goal) {
              goals.push_back(goal);
              ticks_left_ = ticks_to_arrive_;
              failed_ = false;
              return !reject_sends;
            },
        .goal_reached =
            [this] {
              if (failed_ || always_fail) {
                return false;
              }
              if (ticks_left_ > 0) {
                --ticks_left_;
                return false;
              }
              return true;
            },
        .navigation_failed = [this] { return failed_ || always_fail; },
    };
  }

  void fail_current_leg() { failed_ = true; }
  void recover() { failed_ = false; }

  std::vector<geometry::Vec2<double>> goals;
  bool reject_sends{false};
  bool always_fail{false};  ///< persistent fault: survives goal re-sends

 private:
  int ticks_to_arrive_;
  int ticks_left_{0};
  bool failed_{false};
};

Status run_to_completion(WaypointMission& mission, int max_ticks = 100) {
  Status status = Status::kRunning;
  for (int i = 0; i < max_ticks && status == Status::kRunning; ++i) {
    status = mission.tick();
  }
  return status;
}

TEST(WaypointMissionTest, EmptyMissionSucceedsImmediately) {
  FakeNavigator nav;
  WaypointMission mission{{}, nav.callbacks()};
  EXPECT_EQ(mission.tick(), Status::kSuccess);
  EXPECT_EQ(mission.waypoint_count(), 0U);
}

TEST(WaypointMissionTest, VisitsAllWaypointsInOrder) {
  FakeNavigator nav;
  WaypointMission mission{{{1.0, 0.0}, {2.0, 1.0}, {0.0, 2.0}}, nav.callbacks()};

  EXPECT_EQ(run_to_completion(mission), Status::kSuccess);
  ASSERT_EQ(nav.goals.size(), 3U);
  EXPECT_DOUBLE_EQ(nav.goals[0].x, 1.0);
  EXPECT_DOUBLE_EQ(nav.goals[1].x, 2.0);
  EXPECT_DOUBLE_EQ(nav.goals[2].y, 2.0);
  EXPECT_EQ(mission.current_waypoint(), mission.waypoint_count());
}

TEST(WaypointMissionTest, ReportsRunningWhileNavigating) {
  FakeNavigator nav{5};
  WaypointMission mission{{{1.0, 0.0}}, nav.callbacks()};
  EXPECT_EQ(mission.tick(), Status::kRunning);
  EXPECT_EQ(mission.current_waypoint(), 0U);
}

TEST(WaypointMissionTest, RetriesFailedLegAndRecovers) {
  FakeNavigator nav;
  WaypointMission mission{{{1.0, 0.0}}, nav.callbacks()};

  EXPECT_EQ(mission.tick(), Status::kRunning);
  nav.fail_current_leg();
  EXPECT_EQ(mission.tick(), Status::kRunning);  // failure consumed by retry
  nav.recover();
  EXPECT_EQ(run_to_completion(mission), Status::kSuccess);
  // The goal was re-sent for the retry attempt.
  EXPECT_EQ(nav.goals.size(), 2U);
}

TEST(WaypointMissionTest, AbortsAfterAttemptBudgetExhausted) {
  FakeNavigator nav;
  WaypointMission mission{
      {{1.0, 0.0}, {2.0, 0.0}}, nav.callbacks(), {.max_attempts_per_waypoint = 2}};

  EXPECT_EQ(mission.tick(), Status::kRunning);
  nav.always_fail = true;
  Status status = Status::kRunning;
  for (int i = 0; i < 10 && status == Status::kRunning; ++i) {
    status = mission.tick();
  }
  EXPECT_EQ(status, Status::kFailure);
  // Both attempts targeted the first waypoint; the second was never reached.
  EXPECT_EQ(nav.goals.size(), 2U);
  EXPECT_DOUBLE_EQ(nav.goals[1].x, 1.0);
  EXPECT_EQ(mission.current_waypoint(), 0U);
}

TEST(WaypointMissionTest, RejectedGoalSendCountsAsAttempt) {
  FakeNavigator nav;
  nav.reject_sends = true;
  WaypointMission mission{{{1.0, 0.0}}, nav.callbacks(), {.max_attempts_per_waypoint = 3}};

  EXPECT_EQ(mission.tick(), Status::kFailure);
  EXPECT_EQ(nav.goals.size(), 3U);
}

TEST(WaypointMissionTest, HaltRestartsFromFirstWaypoint) {
  FakeNavigator nav{3};
  WaypointMission mission{{{1.0, 0.0}, {2.0, 0.0}}, nav.callbacks()};

  EXPECT_EQ(mission.tick(), Status::kRunning);
  mission.halt();
  EXPECT_EQ(mission.current_waypoint(), 0U);

  EXPECT_EQ(run_to_completion(mission), Status::kSuccess);
  // First goal was sent twice: before and after the halt.
  ASSERT_EQ(nav.goals.size(), 3U);
  EXPECT_DOUBLE_EQ(nav.goals[0].x, 1.0);
  EXPECT_DOUBLE_EQ(nav.goals[1].x, 1.0);
  EXPECT_DOUBLE_EQ(nav.goals[2].x, 2.0);
}

}  // namespace
}  // namespace ars::core::mission
