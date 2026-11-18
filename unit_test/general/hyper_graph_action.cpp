// SPDX-FileCopyrightText: 2026 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#include "g2o/core/hyper_graph_action.h"

#include <memory>
#include <string>

#include "gmock/gmock.h"
#include "gtest/gtest.h"

using ::testing::SizeIs;

namespace {

//! Action of a collection that counts how often it was destroyed.
class CountingAction : public g2o::HyperGraphElementAction {
 public:
  CountingAction(const std::string& name, const std::string& typeName,
                 int& destructions)
      : g2o::HyperGraphElementAction(typeName), _destructions(destructions) {
    _name = name;
  }

  ~CountingAction() override { ++_destructions; }

 private:
  int& _destructions;
};

}  // namespace

/**
 * A collection holds a single action per type. A second action for the same
 * type is rejected, and the collection frees it.
 */
TEST(General, ActionCollectionDeletesARejectedDuplicate) {
  int keptDestructions = 0;
  int rejectedDestructions = 0;
  g2o::HyperGraphElementActionCollection collection("draw");
  auto kept =
      std::make_unique<CountingAction>("draw", "VertexSE3", keptDestructions);
  const CountingAction* keptPointer = kept.get();
  ASSERT_TRUE(collection.registerAction(std::move(kept)));

  EXPECT_FALSE(collection.registerAction(std::make_unique<CountingAction>(
      "draw", "VertexSE3", rejectedDestructions)));

  EXPECT_EQ(keptDestructions, 0);
  EXPECT_EQ(rejectedDestructions, 1);
  EXPECT_EQ(collection.actionMap().at("VertexSE3").get(), keptPointer);
  EXPECT_THAT(collection.actionMap(), SizeIs(1));
}
