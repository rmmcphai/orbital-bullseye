#include <catch2/catch_test_macros.hpp>

#include "core/publisher.hpp"

using bullseye::Publisher;

TEST_CASE("Publisher seqno starts at 0 and increments on publish") {
  Publisher pub;

  // Initial front snapshot has default seqno.
  REQUIRE(pub.published_seqno() == 0);

  auto& back = pub.begin_write();
  back.positions[0][0] = {1.0, 2.0, 3.0};

  const auto s1 = pub.publish(10.0);
  REQUIRE(s1 == 1);
  REQUIRE(pub.published_seqno() == 1);

  auto& back2 = pub.begin_write();
  back2.positions[0][0] = {4.0, 5.0, 6.0};

  const auto s2 = pub.publish(20.0);
  REQUIRE(s2 == 2);
  REQUIRE(pub.published_seqno() == 2);
}

TEST_CASE("Read returns the last published snapshot") {
  Publisher pub;

  // Publish #1
  {
    auto& back = pub.begin_write();
    back.positions[0][0] = {1.0, 2.0, 3.0};
    pub.publish(10.0);
  }

  const auto& front1 = pub.read();
  REQUIRE(front1.seqno == 1);
  REQUIRE(front1.t0 == 10.0);
  REQUIRE(front1.positions[0][0].x == 1.0);
  REQUIRE(front1.positions[0][0].y == 2.0);
  REQUIRE(front1.positions[0][0].z == 3.0);

  // Publish #2 overwrites the *other* buffer
  {
    auto& back = pub.begin_write();
    back.positions[0][0] = {4.0, 5.0, 6.0};
    pub.publish(20.0);
  }

  const auto& front2 = pub.read();
  REQUIRE(front2.seqno == 2);
  REQUIRE(front2.t0 == 20.0);
  REQUIRE(front2.positions[0][0].x == 4.0);
  REQUIRE(front2.positions[0][0].y == 5.0);
  REQUIRE(front2.positions[0][0].z == 6.0);
}

TEST_CASE("Front snapshot is stable until next publish") {
  Publisher pub;

  // Publish #1
  {
    auto& back = pub.begin_write();
    back.positions[0][0] = {1.0, 1.0, 1.0};
    pub.publish(10.0);
  }

  const auto& front_before = pub.read();

  // Modify back buffer, but do not publish yet.
  {
    auto& back = pub.begin_write();
    back.positions[0][0] = {9.0, 9.0, 9.0};
  }

  // Front should remain unchanged.
  const auto& front_after = pub.read();
  REQUIRE(front_after.seqno == front_before.seqno);
  REQUIRE(front_after.t0 == front_before.t0);
  REQUIRE(front_after.positions[0][0].x == 1.0);
  REQUIRE(front_after.positions[0][0].y == 1.0);
  REQUIRE(front_after.positions[0][0].z == 1.0);

  // Now publish and front changes.
  pub.publish(20.0);
  const auto& front_published = pub.read();
  REQUIRE(front_published.seqno == 2);
  REQUIRE(front_published.positions[0][0].x == 9.0);
}
