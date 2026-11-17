// SPDX-FileCopyrightText: 2026 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#include <sstream>

#include "g2o/core/factory.h"
#include "g2o/types/slam2d/vertex_se2.h"
#include "gmock/gmock.h"
#include "gtest/gtest.h"

/**
 * The listing of the registered types is output for the caller, so every
 * type name goes to the stream the caller passes.
 */
TEST(General, FactoryPrintsRegisteredTypesToTheGivenStream) {
  // A vertex of the library makes sure that its types are registered.
  g2o::VertexSE2 vertex;
  std::ostringstream listing;

  g2o::Factory::instance()->printRegisteredTypes(listing, true);

  EXPECT_THAT(listing.str(), testing::HasSubstr("#\tVERTEX_SE2\n"));
}
