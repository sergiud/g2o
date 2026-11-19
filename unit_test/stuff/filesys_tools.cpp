// SPDX-FileCopyrightText: 2026 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#include "g2o/stuff/filesys_tools.h"

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <string>
#include <string_view>
#include <vector>

#include "gmock/gmock.h"
#include "gtest/gtest.h"

using ::testing::ElementsAre;

namespace {

namespace fs = std::filesystem;

//! Directory below the temporary directory of the test that it removes.
class FilesByPattern : public ::testing::Test {
 protected:
  //! Creates the directory with the given name and an empty file in it.
  fs::path createFile(const std::string& directory, const std::string& name) {
    const fs::path path = root_ / directory / name;
    fs::create_directories(path.parent_path());
    std::ofstream{path};
    return path;
  }

  void TearDown() override { fs::remove_all(root_); }

  const fs::path root_ =
      fs::path{::testing::TempDir()} /
      ::testing::UnitTest::GetInstance()->current_test_info()->name();
};

}  // namespace

/**
 * The plugins of an installation below a directory whose name contains a
 * space are found like any other.
 */
TEST_F(FilesByPattern, MatchesInADirectoryWithASpace) {
  const fs::path plugin = createFile("with space", "lib_types_a.so");
  createFile("with space", "lib_solver_a.so");

  EXPECT_THAT(g2o::getFilesByPattern(
                  (plugin.parent_path() / "*_types_*.so").string().c_str()),
              ElementsAre(plugin.string()));
}

/**
 * The directory comes from environment variables, so its name is taken
 * literally and never runs a command.
 */
TEST_F(FilesByPattern, TakesTheDirectoryLiterally) {
  const fs::path marker = root_ / "marker";
  const fs::path plugin =
      createFile("$(touch " + marker.string() + ")", "lib_types_a.so");

  EXPECT_THAT(g2o::getFilesByPattern(
                  (plugin.parent_path() / "*_types_*.so").string().c_str()),
              ElementsAre(plugin.string()));
  EXPECT_FALSE(fs::exists(marker));
}

//! Matches come in lexicographic order, so plugins load deterministically.
TEST_F(FilesByPattern, ReturnsTheMatchesInOrder) {
  // Files created out of order defeat a directory that lists them as created
  // or in reverse.
  std::vector<std::string> expected;
  for (const char letter : std::string_view{"ecgahbfd"}) {
    expected.push_back(
        createFile("plugins", std::string{"lib_types_"} + letter + ".so")
            .string());
  }
  std::ranges::sort(expected);

  EXPECT_THAT(g2o::getFilesByPattern(
                  (root_ / "plugins" / "lib_types_?.so").string().c_str()),
              ::testing::ContainerEq(expected));
}

//! A trailing '*' also matches nothing, such as a missing version suffix.
TEST_F(FilesByPattern, MatchesAnEmptySuffix) {
  const fs::path unversioned = createFile("plugins", "lib_types_a.so");
  const fs::path versioned = createFile("plugins", "lib_types_a.so.1");

  EXPECT_THAT(g2o::getFilesByPattern(
                  (root_ / "plugins" / "*_types_*.so*").string().c_str()),
              ElementsAre(unversioned.string(), versioned.string()));
}

/**
 * The extension belongs to the name of the file, so a dot in the name of a
 * directory starts none.
 */
TEST(FileExtension, IgnoresADotInADirectory) {
  EXPECT_EQ(g2o::getFileExtension("results.d/plot"), "");
  EXPECT_EQ(g2o::getPureFilename("results.d/plot"), "results.d/plot");
  EXPECT_EQ(g2o::changeFileExtension("results.d/plot", "txt"),
            "results.d/plot");
}

TEST(FileExtension, SplitsTheNameOfTheFile) {
  EXPECT_EQ(g2o::getFileExtension("results.d/plot.dat"), "dat");
  EXPECT_EQ(g2o::getPureFilename("results.d/plot.dat"), "results.d/plot");
  EXPECT_EQ(g2o::changeFileExtension("results.d/plot.dat", "txt"),
            "results.d/plot.txt");
}
