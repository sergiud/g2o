#ifndef G2O_ABI_H
#define G2O_ABI_H

#include <Eigen/Core>

//! The name of the g2o namespace.
#define G2O_LIBRARY_NAMESPACE_NAME g2o
//! Concatenates two preprocessor values (without expansion).
#define G2O_PPCAT(a, b) a ## b
//! Concatenates two preprocessor values (with expansion).
#define G2O_PPCAT2(a, b) G2O_PPCAT(a, b)
//! Used matrix alignment.
#define G2O_ALIGNMENT EIGEN_MAX_ALIGN_BYTES
//! The name of the inner library namespace which defines the ABI.
#define G2O_ABI_NAMESPACE_NAME G2O_PPCAT2(G2O_PPCAT2(eigen_requires_, G2O_ALIGNMENT), _bytes_alignment)

//! Starts the inner ABI specific namespace.
#define G2O_START_ABI_NAMESPACE \
    namespace G2O_ABI_NAMESPACE_NAME {

//! End the inner ABI specific namespace.
#define G2O_END_ABI_NAMESPACE }

//! Starts the outer library specific namespace.
#define G2O_START_LIBRARY_NAMESPACE \
    namespace G2O_LIBRARY_NAMESPACE_NAME {

//! Ends the outer library specific namespace.
#define G2O_END_LIBRARY_NAMESPACE }

//! Starts the ABI specific library namespaces.
#define G2O_START_NAMESPACE \
    G2O_START_LIBRARY_NAMESPACE \
    G2O_START_ABI_NAMESPACE

//! Ends the ABI specific library namespaces.
#define G2O_END_NAMESPACE \
    G2O_END_LIBRARY_NAMESPACE \
    G2O_END_ABI_NAMESPACE

G2O_START_LIBRARY_NAMESPACE
G2O_START_ABI_NAMESPACE
G2O_END_ABI_NAMESPACE
using namespace G2O_LIBRARY_NAMESPACE_NAME::G2O_ABI_NAMESPACE_NAME;
G2O_END_LIBRARY_NAMESPACE

#endif
