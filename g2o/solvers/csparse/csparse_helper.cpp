// SPDX-FileCopyrightText: 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: LGPL-3.0-or-later

#include <g2o/solvers/csparse/csparse_helper.h>
#include <g2o/stuff/logger.h>
#include <g2o/stuff/macros.h>
#include <g2o/stuff/sparse_helper.h>

#include <cassert>
#include <memory>

G2O_START_NAMESPACE
namespace csparse_extension {

/**
 * Originally from CSparse, avoid memory re-allocations by giving workspace
 * pointers CSparse: Copyright (c) 2006-2011, Timothy A. Davis.
 */
int cs_cholsolsymb(const cs* A, double* b, const css* S, double* x, int* work) {
  csn* N;
  int n, ok;
  if (!CS_CSC(A) || (b == nullptr) || (S == nullptr) || (x == nullptr)) {
    ngx::Log(ngx::Error,
             "{}: the matrix, the right-hand side, the symbolic "
             "factorization and the solution must all be given",
             __func__);
    return (0); /* check inputs */
  }
  n = A->n;
  N = cs_chol_workspace(A, S, work, x); /* numeric Cholesky factorization */
  // The caller reports a failed factorization, which the result tells it.
  ok = static_cast<int>(N != nullptr);
  if (ok != 0) {
    cs_ipvec(S->pinv, b, x, n); /* x = P*b */
    cs_lsolve(N->L, x);         /* x = L\x */
    cs_ltsolve(N->L, x);        /* x = L'\x */
    cs_pvec(S->pinv, x, b, n);  /* b = P'*x */
  }
  cs_nfree(N);
  return (ok);
}

/**
 * Originally from CSparse, avoid memory re-allocations by giving workspace
 * pointers CSparse: Copyright (c) 2006-2011, Timothy A. Davis.
 */
/* L = chol (A, [pinv parent cp]), pinv is optional */
csn* cs_chol_workspace(const cs* A, const css* S, int* cin, double* xin) {
  double d, lki, *Lx, *x, *Cx;
  int top, i, p, k, n, *Li, *Lp, *cp, *pinv, *s, *c, *parent, *Cp, *Ci;
  cs *L, *C, *E;
  csn* N;
  if (!CS_CSC(A) || (S == nullptr) || (S->cp == nullptr) ||
      (S->parent == nullptr))
    return (nullptr);
  n = A->n;
  N = (csn*)cs_calloc(1, sizeof(csn)); /* allocate result */
  c = cin;                             /* get int workspace */
  x = xin;                             /* get double workspace */
  cp = S->cp;
  pinv = S->pinv;
  parent = S->parent;
  C = pinv != nullptr ? cs_symperm(A, pinv, 1) : ((cs*)A);
  E = pinv != nullptr ? C : nullptr; /* E is alias for A, or a copy E=A(p,p) */
  if ((N == nullptr) || (c == nullptr) || (x == nullptr) || (C == nullptr))
    return (cs_ndone(N, E, nullptr, nullptr, 0));
  s = c + n;
  Cp = C->p;
  Ci = C->i;
  Cx = C->x;
  N->L = L = cs_spalloc(n, n, cp[n], 1, 0); /* allocate result */
  if (L == nullptr) return (cs_ndone(N, E, nullptr, nullptr, 0));
  Lp = L->p;
  Li = L->i;
  Lx = L->x;
  for (k = 0; k < n; k++) Lp[k] = c[k] = cp[k];
  for (k = 0; k < n; k++) /* compute L(k,:) for L*L' = C */
  {
    /* --- Nonzero pattern of L(k,:) ------------------------------------ */
    top = cs_ereach(C, k, parent, s, c); /* find pattern of L(k,:) */
    x[k] = 0;                            /* x (0:k) is now zero */
    for (p = Cp[k]; p < Cp[k + 1]; p++)  /* x = full(triu(C(:,k))) */
    {
      if (Ci[p] <= k) x[Ci[p]] = Cx[p];
    }
    d = x[k]; /* d = C(k,k) */
    x[k] = 0; /* clear x for k+1st iteration */
    /* --- Triangular solve --------------------------------------------- */
    for (; top < n; top++) /* solve L(0:k-1,0:k-1) * x = C(:,k) */
    {
      i = s[top];             /* s [top..n-1] is pattern of L(k,:) */
      lki = x[i] / Lx[Lp[i]]; /* L(k,i) = x (i) / L(i,i) */
      x[i] = 0;               /* clear x for k+1st iteration */
      for (p = Lp[i] + 1; p < c[i]; p++) {
        x[Li[p]] -= Lx[p] * lki;
      }
      d -= lki * lki; /* d = d - L(k,i)*L(k,i) */
      p = c[i]++;
      Li[p] = k; /* store L(k,i) in column i */
      Lx[p] = lki;
    }
    /* --- Compute L(k,k) ----------------------------------------------- */
    /* not pos def, which a NaN is not either */
    if (!(d > 0)) return (cs_ndone(N, E, nullptr, nullptr, 0));
    p = c[k]++;
    Li[p] = k; /* store L(k,k) = sqrt (d) in column k */
    Lx[p] = sqrt(d);
  }
  Lp[n] = cp[n]; /* finalize L */
  return (
      cs_ndone(N, E, nullptr, nullptr, 1)); /* success: free E,s,x; return N */
}

bool writeCs2Octave(const char* filename, const cs* A, bool upperTriangular) {
  if (A->nz == -1) {
    return writeCCSMatrix(filename, A->m, A->n, A->p, A->i, A->x,
                          upperTriangular);
  }
  // CSparse sums the duplicates of triplets while compressing them, as
  // Octave does when it reads the entries.
  const std::unique_ptr<cs, decltype(&cs_spfree)> compressed(cs_compress(A),
                                                             &cs_spfree);
  if (compressed == nullptr) {
    return false;
  }
  return writeCCSMatrix(filename, compressed->m, compressed->n, compressed->p,
                        compressed->i, compressed->x, upperTriangular);
}

}  // namespace csparse_extension
G2O_END_NAMESPACE
