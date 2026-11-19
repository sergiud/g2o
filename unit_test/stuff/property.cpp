// SPDX-FileCopyrightText: 2026 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#include "g2o/stuff/property.h"

#include "gtest/gtest.h"

namespace {

//! Map of the properties a and b, which hold 1 and 2.
class PropertyMapUpdate : public ::testing::Test {
 protected:
  void SetUp() override {
    _a = _map.makeProperty<g2o::IntProperty>("a", 1);
    _b = _map.makeProperty<g2o::IntProperty>("b", 2);
  }

  g2o::PropertyMap _map;
  g2o::IntProperty* _a = nullptr;
  g2o::IntProperty* _b = nullptr;
};

}  // namespace

//! An unknown name fails the update but not the updates of the others.
TEST_F(PropertyMapUpdate, UpdatesTheKnownProperties) {
  EXPECT_FALSE(_map.updateMapFromString("a=3,unknown=4,b=5"));

  EXPECT_EQ(_a->value(), 3);
  EXPECT_EQ(_b->value(), 5);
}

//! A value of another type fails the update and keeps the value before.
TEST_F(PropertyMapUpdate, KeepsAPropertyWhoseValueIsInvalid) {
  EXPECT_FALSE(_map.updateMapFromString("a=abc"));

  EXPECT_EQ(_a->value(), 1);
}

//! A pair without a value fails the update.
TEST_F(PropertyMapUpdate, RejectsAPairWithoutValue) {
  EXPECT_FALSE(_map.updateMapFromString("a=7,b"));

  EXPECT_EQ(_a->value(), 7);
}

//! Whitespace around a value, such as before a separator, is no part of it.
TEST(Property, ParsesAValueFollowedByWhitespace) {
  g2o::Property<double> property("lambda", 0.);

  ASSERT_TRUE(property.fromString("1.5 "));

  EXPECT_EQ(property.value(), 1.5);
}
