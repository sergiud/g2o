// SPDX-FileCopyrightText: 2026 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#include <memory>
#include <string>

#include "gtest/gtest.h"
#include "g2o/core/hyper_graph_action.h"

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
 * type is rejected and stays with the caller, who still owns it, such as the
 * proxies that register the actions of the type libraries.
 */
TEST(General, ActionCollectionRejectsDuplicateTypeWithoutDeletingIt) {
  int destructions = 0;
  g2o::HyperGraphElementActionCollection collection("draw");

  ASSERT_TRUE(collection.registerAction(
      new CountingAction("draw", "VertexSE3", destructions)));

  auto duplicate =
      std::make_unique<CountingAction>("draw", "VertexSE3", destructions);

  EXPECT_FALSE(collection.registerAction(duplicate.get()));
  EXPECT_EQ(destructions, 0);
  EXPECT_EQ(collection.actionMap().size(), 1U);
  EXPECT_NE(collection.actionMap().at("VertexSE3").get(), duplicate.get());

  // A collection that deleted the duplicate leaves nothing to own.
  if (destructions != 0) {
    static_cast<void>(duplicate.release());
  }
}
