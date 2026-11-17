// g2o - General Graph Optimization
// Copyright (C) 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the
//   documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
// IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
// TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
// PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
// TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
// LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
// NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


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
