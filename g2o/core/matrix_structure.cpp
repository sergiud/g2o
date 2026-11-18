// SPDX-FileCopyrightText: 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#include <g2o/core/matrix_structure.h>

#include <algorithm>
#include <fstream>
#include <memory>
#include <string>
#include <vector>
using namespace std;

G2O_START_NAMESPACE

struct ColSort {
  bool operator()(const pair<int, int>& e1, const pair<int, int>& e2) const {
    return e1.second < e2.second ||
           (e1.second == e2.second && e1.first < e2.first);
  }
};

MatrixStructure::MatrixStructure() : n(0), m(0), maxN(0), maxNz(0) {}

MatrixStructure::~MatrixStructure() { free(); }

void MatrixStructure::alloc(int n_, int nz) {
  if (n == 0) {
    maxN = n = n_;
    maxNz = nz;
    Ap = std::make_unique<int[]>(maxN + 1);
    Aii = std::make_unique<int[]>(maxNz);
  } else {
    n = n_;
    if (maxNz < nz) {
      maxNz = 2 * nz;
      Aii = std::make_unique<int[]>(maxNz);
    }
    if (maxN < n) {
      maxN = 2 * n;
      Ap = std::make_unique<int[]>(maxN + 1);
    }
  }
}

void MatrixStructure::free() {
  n = 0;
  m = 0;
  maxN = 0;
  maxNz = 0;
  Aii.reset();
  Ap.reset();
}

bool MatrixStructure::write(const char* filename) const {
  const int& cols = n;
  const int& rows = m;

  string name = filename;
  std::string::size_type lastDot = name.find_last_of('.');
  if (lastDot != std::string::npos) name = name.substr(0, lastDot);

  vector<pair<int, int> > entries;
  for (int i = 0; i < cols; ++i) {
    const int& rbeg = Ap[i];
    const int& rend = Ap[i + 1];
    for (int j = rbeg; j < rend; ++j) {
      entries.push_back(make_pair(Aii[j], i));
      if (Aii[j] != i) entries.push_back(make_pair(i, Aii[j]));
    }
  }

  sort(entries.begin(), entries.end(), ColSort());

  std::ofstream fout(filename);
  fout << "# name: " << name << std::endl;
  fout << "# type: sparse matrix" << std::endl;
  fout << "# nnz: " << entries.size() << std::endl;
  fout << "# rows: " << rows << std::endl;
  fout << "# columns: " << cols << std::endl;
  for (vector<pair<int, int> >::const_iterator it = entries.begin();
       it != entries.end(); ++it) {
    const pair<int, int>& entry = *it;
    fout << entry.first << " " << entry.second << " 0"
         << std::endl;  // write a constant value of 0
  }

  return fout.good();
}

G2O_END_NAMESPACE
