cmake_minimum_required(VERSION 2.8)
project(denoiser)

set(CMAKE_CXX_FLAGS "-std=c++11 -pedantic-errors")
set(CMAKE_CXX_STANDARD 11)
include_directories(.)
include_directories(Denoising)
include_directories(Denoising/Algorithms)
include_directories(Eigen)
include_directories(Eigen/src)
include_directories(Eigen/src/Cholesky)
include_directories(Eigen/src/CholmodSupport)
include_directories(Eigen/src/Core)
include_directories(Eigen/src/Core/arch)
include_directories(Eigen/src/Core/arch/AltiVec)
include_directories(Eigen/src/Core/arch/Default)
include_directories(Eigen/src/Core/arch/NEON)
include_directories(Eigen/src/Core/arch/SSE)
include_directories(Eigen/src/Core/products)
include_directories(Eigen/src/Core/util)
include_directories(Eigen/src/Eigen2Support)
include_directories(Eigen/src/Eigen2Support/Geometry)
include_directories(Eigen/src/Eigenvalues)
include_directories(Eigen/src/Geometry)
include_directories(Eigen/src/Geometry/arch)
include_directories(Eigen/src/Householder)
include_directories(Eigen/src/IterativeLinearSolvers)
include_directories(Eigen/src/Jacobi)
include_directories(Eigen/src/LU)
include_directories(Eigen/src/LU/arch)
include_directories(Eigen/src/MetisSupport)
include_directories(Eigen/src/misc)
include_directories(Eigen/src/OrderingMethods)
include_directories(Eigen/src/PardisoSupport)
include_directories(Eigen/src/PaStiXSupport)
include_directories(Eigen/src/plugins)
include_directories(Eigen/src/QR)
include_directories(Eigen/src/SparseCholesky)
include_directories(Eigen/src/SparseCore)
include_directories(Eigen/src/SparseLU)
include_directories(Eigen/src/SparseQR)
include_directories(Eigen/src/SPQRSupport)
include_directories(Eigen/src/StlSupport)
include_directories(Eigen/src/SuperLUSupport)
include_directories(Eigen/src/SVD)
include_directories(Eigen/src/UmfPackSupport)
include_directories(OpenMesh/Core/Geometry)
include_directories(OpenMesh/Core/IO)
include_directories(OpenMesh/Core/IO/exporter)
include_directories(OpenMesh/Core/IO/importer)
include_directories(OpenMesh/Core/IO/reader)
include_directories(OpenMesh/Core/IO/writer)
include_directories(OpenMesh/Core/Mesh)
include_directories(OpenMesh/Core/Mesh/gen)
include_directories(OpenMesh/Core/System)
include_directories(OpenMesh/Core/Templates)
include_directories(OpenMesh/Core/Utils)

add_executable(denoiser
        Denoising/Algorithms/BilateralMeshDenoising.cpp
        Denoising/Algorithms/BilateralMeshDenoising.h
        Denoising/Algorithms/BilateralNormalFilteringForMeshDenoising.cpp
        Denoising/Algorithms/BilateralNormalFilteringForMeshDenoising.h
        Denoising/Algorithms/FastAndEffectiveFeaturePreservingMeshDenoising.cpp
        Denoising/Algorithms/FastAndEffectiveFeaturePreservingMeshDenoising.h
        Denoising/Algorithms/GuidedMeshNormalFiltering.cpp
        Denoising/Algorithms/GuidedMeshNormalFiltering.h
        Denoising/Algorithms/meshdenoisebase.cpp
        Denoising/Algorithms/meshdenoisebase.h
        Denoising/Algorithms/MeshDenoisingBase.cpp
        Denoising/Algorithms/MeshDenoisingBase.h
        Denoising/Algorithms/MeshDenoisingViaL0Minimization.cpp
        Denoising/Algorithms/MeshDenoisingViaL0Minimization.h
        Denoising/Algorithms/Noise.cpp
        Denoising/Algorithms/Noise.h
        Denoising/Algorithms/NonIterativeFeaturePreservingMeshFiltering.cpp
        Denoising/Algorithms/NonIterativeFeaturePreservingMeshFiltering.h
        Denoising/datamanager.cpp
        Denoising/datamanager.h
        Denoising/mesh.h
        Denoising/parameterset.cpp
        Denoising/parameterset.h
        Eigen/src/Cholesky/LDLT.h
        Eigen/src/Cholesky/LLT.h
        Eigen/src/Cholesky/LLT_MKL.h
        Eigen/src/CholmodSupport/CholmodSupport.h
        Eigen/src/Core/arch/AltiVec/Complex.h
        Eigen/src/Core/arch/AltiVec/PacketMath.h
        Eigen/src/Core/arch/Default/Settings.h
        Eigen/src/Core/arch/NEON/Complex.h
        Eigen/src/Core/arch/NEON/PacketMath.h
        Eigen/src/Core/arch/SSE/Complex.h
        Eigen/src/Core/arch/SSE/MathFunctions.h
        Eigen/src/Core/arch/SSE/PacketMath.h
        Eigen/src/Core/products/CoeffBasedProduct.h
        Eigen/src/Core/products/GeneralBlockPanelKernel.h
        Eigen/src/Core/products/GeneralMatrixMatrix.h
        Eigen/src/Core/products/GeneralMatrixMatrix_MKL.h
        Eigen/src/Core/products/GeneralMatrixMatrixTriangular.h
        Eigen/src/Core/products/GeneralMatrixMatrixTriangular_MKL.h
        Eigen/src/Core/products/GeneralMatrixVector.h
        Eigen/src/Core/products/GeneralMatrixVector_MKL.h
        Eigen/src/Core/products/Parallelizer.h
        Eigen/src/Core/products/SelfadjointMatrixMatrix.h
        Eigen/src/Core/products/SelfadjointMatrixMatrix_MKL.h
        Eigen/src/Core/products/SelfadjointMatrixVector.h
        Eigen/src/Core/products/SelfadjointMatrixVector_MKL.h
        Eigen/src/Core/products/SelfadjointProduct.h
        Eigen/src/Core/products/SelfadjointRank2Update.h
        Eigen/src/Core/products/TriangularMatrixMatrix.h
        Eigen/src/Core/products/TriangularMatrixMatrix_MKL.h
        Eigen/src/Core/products/TriangularMatrixVector.h
        Eigen/src/Core/products/TriangularMatrixVector_MKL.h
        Eigen/src/Core/products/TriangularSolverMatrix.h
        Eigen/src/Core/products/TriangularSolverMatrix_MKL.h
        Eigen/src/Core/products/TriangularSolverVector.h
        Eigen/src/Core/util/BlasUtil.h
        Eigen/src/Core/util/Constants.h
        Eigen/src/Core/util/DisableStupidWarnings.h
        Eigen/src/Core/util/ForwardDeclarations.h
        Eigen/src/Core/util/Macros.h
        Eigen/src/Core/util/Memory.h
        Eigen/src/Core/util/Meta.h
        Eigen/src/Core/util/MKL_support.h
        Eigen/src/Core/util/NonMPL2.h
        Eigen/src/Core/util/ReenableStupidWarnings.h
        Eigen/src/Core/util/StaticAssert.h
        Eigen/src/Core/util/XprHelper.h
        Eigen/src/Core/Array.h
        Eigen/src/Core/ArrayBase.h
        Eigen/src/Core/ArrayWrapper.h
        Eigen/src/Core/Assign.h
        Eigen/src/Core/Assign_MKL.h
        Eigen/src/Core/BandMatrix.h
        Eigen/src/Core/Block.h
        Eigen/src/Core/BooleanRedux.h
        Eigen/src/Core/CommaInitializer.h
        Eigen/src/Core/CoreIterators.h
        Eigen/src/Core/CwiseBinaryOp.h
        Eigen/src/Core/CwiseNullaryOp.h
        Eigen/src/Core/CwiseUnaryOp.h
        Eigen/src/Core/CwiseUnaryView.h
        Eigen/src/Core/DenseBase.h
        Eigen/src/Core/DenseCoeffsBase.h
        Eigen/src/Core/DenseStorage.h
        Eigen/src/Core/Diagonal.h
        Eigen/src/Core/DiagonalMatrix.h
        Eigen/src/Core/DiagonalProduct.h
        Eigen/src/Core/Dot.h
        Eigen/src/Core/EigenBase.h
        Eigen/src/Core/Flagged.h
        Eigen/src/Core/ForceAlignedAccess.h
        Eigen/src/Core/Functors.h
        Eigen/src/Core/Fuzzy.h
        Eigen/src/Core/GeneralProduct.h
        Eigen/src/Core/GenericPacketMath.h
        Eigen/src/Core/GlobalFunctions.h
        Eigen/src/Core/IO.h
        Eigen/src/Core/Map.h
        Eigen/src/Core/MapBase.h
        Eigen/src/Core/MathFunctions.h
        Eigen/src/Core/Matrix.h
        Eigen/src/Core/MatrixBase.h
        Eigen/src/Core/NestByValue.h
        Eigen/src/Core/NoAlias.h
        Eigen/src/Core/NumTraits.h
        Eigen/src/Core/PermutationMatrix.h
        Eigen/src/Core/PlainObjectBase.h
        Eigen/src/Core/ProductBase.h
        Eigen/src/Core/Random.h
        Eigen/src/Core/Redux.h
        Eigen/src/Core/Ref.h
        Eigen/src/Core/Replicate.h
        Eigen/src/Core/ReturnByValue.h
        Eigen/src/Core/Reverse.h
        Eigen/src/Core/Select.h
        Eigen/src/Core/SelfAdjointView.h
        Eigen/src/Core/SelfCwiseBinaryOp.h
        Eigen/src/Core/SolveTriangular.h
        Eigen/src/Core/StableNorm.h
        Eigen/src/Core/Stride.h
        Eigen/src/Core/Swap.h
        Eigen/src/Core/Transpose.h
        Eigen/src/Core/Transpositions.h
        Eigen/src/Core/TriangularMatrix.h
        Eigen/src/Core/VectorBlock.h
        Eigen/src/Core/VectorwiseOp.h
        Eigen/src/Core/Visitor.h
        Eigen/src/Eigen2Support/Geometry/AlignedBox.h
        Eigen/src/Eigen2Support/Geometry/All.h
        Eigen/src/Eigen2Support/Geometry/AngleAxis.h
        Eigen/src/Eigen2Support/Geometry/Hyperplane.h
        Eigen/src/Eigen2Support/Geometry/ParametrizedLine.h
        Eigen/src/Eigen2Support/Geometry/Quaternion.h
        Eigen/src/Eigen2Support/Geometry/Rotation2D.h
        Eigen/src/Eigen2Support/Geometry/RotationBase.h
        Eigen/src/Eigen2Support/Geometry/Scaling.h
        Eigen/src/Eigen2Support/Geometry/Transform.h
        Eigen/src/Eigen2Support/Geometry/Translation.h
        Eigen/src/Eigen2Support/Block.h
        Eigen/src/Eigen2Support/Cwise.h
        Eigen/src/Eigen2Support/CwiseOperators.h
        Eigen/src/Eigen2Support/Lazy.h
        Eigen/src/Eigen2Support/LeastSquares.h
        Eigen/src/Eigen2Support/LU.h
        Eigen/src/Eigen2Support/Macros.h
        Eigen/src/Eigen2Support/MathFunctions.h
        Eigen/src/Eigen2Support/Memory.h
        Eigen/src/Eigen2Support/Meta.h
        Eigen/src/Eigen2Support/Minor.h
        Eigen/src/Eigen2Support/QR.h
        Eigen/src/Eigen2Support/SVD.h
        Eigen/src/Eigen2Support/TriangularSolver.h
        Eigen/src/Eigen2Support/VectorBlock.h
        Eigen/src/Eigenvalues/ComplexEigenSolver.h
        Eigen/src/Eigenvalues/ComplexSchur.h
        Eigen/src/Eigenvalues/ComplexSchur_MKL.h
        Eigen/src/Eigenvalues/EigenSolver.h
        Eigen/src/Eigenvalues/GeneralizedEigenSolver.h
        Eigen/src/Eigenvalues/GeneralizedSelfAdjointEigenSolver.h
        Eigen/src/Eigenvalues/HessenbergDecomposition.h
        Eigen/src/Eigenvalues/MatrixBaseEigenvalues.h
        Eigen/src/Eigenvalues/RealQZ.h
        Eigen/src/Eigenvalues/RealSchur.h
        Eigen/src/Eigenvalues/RealSchur_MKL.h
        Eigen/src/Eigenvalues/SelfAdjointEigenSolver.h
        Eigen/src/Eigenvalues/SelfAdjointEigenSolver_MKL.h
        Eigen/src/Eigenvalues/Tridiagonalization.h
        Eigen/src/Geometry/arch/Geometry_SSE.h
        Eigen/src/Geometry/AlignedBox.h
        Eigen/src/Geometry/AngleAxis.h
        Eigen/src/Geometry/EulerAngles.h
        Eigen/src/Geometry/Homogeneous.h
        Eigen/src/Geometry/Hyperplane.h
        Eigen/src/Geometry/OrthoMethods.h
        Eigen/src/Geometry/ParametrizedLine.h
        Eigen/src/Geometry/Quaternion.h
        Eigen/src/Geometry/Rotation2D.h
        Eigen/src/Geometry/RotationBase.h
        Eigen/src/Geometry/Scaling.h
        Eigen/src/Geometry/Transform.h
        Eigen/src/Geometry/Translation.h
        Eigen/src/Geometry/Umeyama.h
        Eigen/src/Householder/BlockHouseholder.h
        Eigen/src/Householder/Householder.h
        Eigen/src/Householder/HouseholderSequence.h
        Eigen/src/IterativeLinearSolvers/BasicPreconditioners.h
        Eigen/src/IterativeLinearSolvers/BiCGSTAB.h
        Eigen/src/IterativeLinearSolvers/ConjugateGradient.h
        Eigen/src/IterativeLinearSolvers/IncompleteLUT.h
        Eigen/src/IterativeLinearSolvers/IterativeSolverBase.h
        Eigen/src/Jacobi/Jacobi.h
        Eigen/src/LU/arch/Inverse_SSE.h
        Eigen/src/LU/Determinant.h
        Eigen/src/LU/FullPivLU.h
        Eigen/src/LU/Inverse.h
        Eigen/src/LU/PartialPivLU.h
        Eigen/src/LU/PartialPivLU_MKL.h
        Eigen/src/MetisSupport/MetisSupport.h
        Eigen/src/misc/blas.h
        Eigen/src/misc/Image.h
        Eigen/src/misc/Kernel.h
        Eigen/src/misc/Solve.h
        Eigen/src/misc/SparseSolve.h
        Eigen/src/OrderingMethods/Amd.h
        Eigen/src/OrderingMethods/Eigen_Colamd.h
        Eigen/src/OrderingMethods/Ordering.h
        Eigen/src/PardisoSupport/PardisoSupport.h
        Eigen/src/PaStiXSupport/PaStiXSupport.h
        Eigen/src/plugins/ArrayCwiseBinaryOps.h
        Eigen/src/plugins/ArrayCwiseUnaryOps.h
        Eigen/src/plugins/BlockMethods.h
        Eigen/src/plugins/CommonCwiseBinaryOps.h
        Eigen/src/plugins/CommonCwiseUnaryOps.h
        Eigen/src/plugins/MatrixCwiseBinaryOps.h
        Eigen/src/plugins/MatrixCwiseUnaryOps.h
        Eigen/src/QR/ColPivHouseholderQR.h
        Eigen/src/QR/ColPivHouseholderQR_MKL.h
        Eigen/src/QR/FullPivHouseholderQR.h
        Eigen/src/QR/HouseholderQR.h
        Eigen/src/QR/HouseholderQR_MKL.h
        Eigen/src/SparseCholesky/SimplicialCholesky.h
        Eigen/src/SparseCholesky/SimplicialCholesky_impl.h
        Eigen/src/SparseCore/AmbiVector.h
        Eigen/src/SparseCore/CompressedStorage.h
        Eigen/src/SparseCore/ConservativeSparseSparseProduct.h
        Eigen/src/SparseCore/MappedSparseMatrix.h
        Eigen/src/SparseCore/SparseBlock.h
        Eigen/src/SparseCore/SparseColEtree.h
        Eigen/src/SparseCore/SparseCwiseBinaryOp.h
        Eigen/src/SparseCore/SparseCwiseUnaryOp.h
        Eigen/src/SparseCore/SparseDenseProduct.h
        Eigen/src/SparseCore/SparseDiagonalProduct.h
        Eigen/src/SparseCore/SparseDot.h
        Eigen/src/SparseCore/SparseFuzzy.h
        Eigen/src/SparseCore/SparseMatrix.h
        Eigen/src/SparseCore/SparseMatrixBase.h
        Eigen/src/SparseCore/SparsePermutation.h
        Eigen/src/SparseCore/SparseProduct.h
        Eigen/src/SparseCore/SparseRedux.h
        Eigen/src/SparseCore/SparseSelfAdjointView.h
        Eigen/src/SparseCore/SparseSparseProductWithPruning.h
        Eigen/src/SparseCore/SparseTranspose.h
        Eigen/src/SparseCore/SparseTriangularView.h
        Eigen/src/SparseCore/SparseUtil.h
        Eigen/src/SparseCore/SparseVector.h
        Eigen/src/SparseCore/SparseView.h
        Eigen/src/SparseCore/TriangularSolver.h
        Eigen/src/SparseLU/SparseLU.h
        Eigen/src/SparseLU/SparseLU_column_bmod.h
        Eigen/src/SparseLU/SparseLU_column_dfs.h
        Eigen/src/SparseLU/SparseLU_copy_to_ucol.h
        Eigen/src/SparseLU/SparseLU_gemm_kernel.h
        Eigen/src/SparseLU/SparseLU_heap_relax_snode.h
        Eigen/src/SparseLU/SparseLU_kernel_bmod.h
        Eigen/src/SparseLU/SparseLU_Memory.h
        Eigen/src/SparseLU/SparseLU_panel_bmod.h
        Eigen/src/SparseLU/SparseLU_panel_dfs.h
        Eigen/src/SparseLU/SparseLU_pivotL.h
        Eigen/src/SparseLU/SparseLU_pruneL.h
        Eigen/src/SparseLU/SparseLU_relax_snode.h
        Eigen/src/SparseLU/SparseLU_Structs.h
        Eigen/src/SparseLU/SparseLU_SupernodalMatrix.h
        Eigen/src/SparseLU/SparseLU_Utils.h
        Eigen/src/SparseLU/SparseLUImpl.h
        Eigen/src/SparseQR/SparseQR.h
        Eigen/src/SPQRSupport/SuiteSparseQRSupport.h
        Eigen/src/StlSupport/details.h
        Eigen/src/StlSupport/StdDeque.h
        Eigen/src/StlSupport/StdList.h
        Eigen/src/StlSupport/StdVector.h
        Eigen/src/SuperLUSupport/SuperLUSupport.h
        Eigen/src/SVD/JacobiSVD.h
        Eigen/src/SVD/JacobiSVD_MKL.h
        Eigen/src/SVD/UpperBidiagonalization.h
        Eigen/src/UmfPackSupport/UmfPackSupport.h
        OpenMesh/Core/Geometry/Config.hh
        OpenMesh/Core/Geometry/LoopSchemeMaskT.hh
        OpenMesh/Core/Geometry/MathDefs.hh
        OpenMesh/Core/Geometry/NormalConeT.cc
        OpenMesh/Core/Geometry/NormalConeT.hh
        OpenMesh/Core/Geometry/Plane3d.hh
        OpenMesh/Core/Geometry/QuadricT.hh
        OpenMesh/Core/Geometry/VectorT.hh
        OpenMesh/Core/Geometry/VectorT_inc.hh
        OpenMesh/Core/IO/exporter/BaseExporter.hh
        OpenMesh/Core/IO/exporter/ExporterT.hh
        OpenMesh/Core/IO/importer/BaseImporter.hh
        OpenMesh/Core/IO/importer/ImporterT.hh
        OpenMesh/Core/IO/reader/BaseReader.cc
        OpenMesh/Core/IO/reader/BaseReader.hh
        OpenMesh/Core/IO/reader/OBJReader.cc
        OpenMesh/Core/IO/reader/OBJReader.hh
        OpenMesh/Core/IO/reader/OFFReader.cc
        OpenMesh/Core/IO/reader/OFFReader.hh
        OpenMesh/Core/IO/reader/OMReader.cc
        OpenMesh/Core/IO/reader/OMReader.hh
        OpenMesh/Core/IO/reader/PLYReader.cc
        OpenMesh/Core/IO/reader/PLYReader.hh
        OpenMesh/Core/IO/reader/STLReader.cc
        OpenMesh/Core/IO/reader/STLReader.hh
        OpenMesh/Core/IO/writer/BaseWriter.cc
        OpenMesh/Core/IO/writer/BaseWriter.hh
        OpenMesh/Core/IO/writer/OBJWriter.cc
        OpenMesh/Core/IO/writer/OBJWriter.hh
        OpenMesh/Core/IO/writer/OFFWriter.cc
        OpenMesh/Core/IO/writer/OFFWriter.hh
        OpenMesh/Core/IO/writer/OMWriter.cc
        OpenMesh/Core/IO/writer/OMWriter.hh
        OpenMesh/Core/IO/writer/PLYWriter.cc
        OpenMesh/Core/IO/writer/PLYWriter.hh
        OpenMesh/Core/IO/writer/STLWriter.cc
        OpenMesh/Core/IO/writer/STLWriter.hh
        OpenMesh/Core/IO/writer/VTKWriter.cc
        OpenMesh/Core/IO/writer/VTKWriter.hh
        OpenMesh/Core/IO/BinaryHelper.cc
        OpenMesh/Core/IO/BinaryHelper.hh
        OpenMesh/Core/IO/IOInstances.hh
        OpenMesh/Core/IO/IOManager.cc
        OpenMesh/Core/IO/IOManager.hh
        OpenMesh/Core/IO/MeshIO.hh
        OpenMesh/Core/IO/OFFFormat.hh
        OpenMesh/Core/IO/OMFormat.cc
        OpenMesh/Core/IO/OMFormat.hh
        OpenMesh/Core/IO/OMFormatT.cc
        OpenMesh/Core/IO/Options.hh
        OpenMesh/Core/IO/SR_binary.hh
        OpenMesh/Core/IO/SR_binary_spec.hh
        OpenMesh/Core/IO/SR_binary_vector_of_bool.inl
        OpenMesh/Core/IO/SR_binary_vector_of_fundamentals.inl
        OpenMesh/Core/IO/SR_binary_vector_of_string.inl
        OpenMesh/Core/IO/SR_rbo.hh
        OpenMesh/Core/IO/SR_store.hh
        OpenMesh/Core/IO/SR_types.hh
        OpenMesh/Core/IO/StoreRestore.hh
        OpenMesh/Core/Mesh/gen/circulators_header.hh
        OpenMesh/Core/Mesh/gen/circulators_template.hh
        OpenMesh/Core/Mesh/gen/footer.hh
        OpenMesh/Core/Mesh/gen/iterators_header.hh
        OpenMesh/Core/Mesh/gen/iterators_template.hh
        OpenMesh/Core/Mesh/ArrayItems.hh
        OpenMesh/Core/Mesh/ArrayKernel.cc
        OpenMesh/Core/Mesh/ArrayKernel.hh
        OpenMesh/Core/Mesh/ArrayKernelT.cc
        OpenMesh/Core/Mesh/AttribKernelT.hh
        OpenMesh/Core/Mesh/Attributes.hh
        OpenMesh/Core/Mesh/BaseKernel.cc
        OpenMesh/Core/Mesh/BaseKernel.hh
        OpenMesh/Core/Mesh/BaseMesh.hh
        OpenMesh/Core/Mesh/Casts.hh
        OpenMesh/Core/Mesh/CirculatorsT.hh
        OpenMesh/Core/Mesh/FinalMeshItemsT.hh
        OpenMesh/Core/Mesh/Handles.hh
        OpenMesh/Core/Mesh/IteratorsT.hh
        OpenMesh/Core/Mesh/PolyConnectivity.cc
        OpenMesh/Core/Mesh/PolyConnectivity.hh
        OpenMesh/Core/Mesh/PolyMesh_ArrayKernelT.hh
        OpenMesh/Core/Mesh/PolyMeshT.cc
        OpenMesh/Core/Mesh/PolyMeshT.hh
        OpenMesh/Core/Mesh/Status.hh
        OpenMesh/Core/Mesh/Traits.hh
        OpenMesh/Core/Mesh/TriConnectivity.cc
        OpenMesh/Core/Mesh/TriConnectivity.hh
        OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh
        OpenMesh/Core/Mesh/TriMeshT.cc
        OpenMesh/Core/Mesh/TriMeshT.hh
        OpenMesh/Core/System/compiler.hh
        OpenMesh/Core/System/config.h
        OpenMesh/Core/System/config.hh
        OpenMesh/Core/System/mostream.hh
        OpenMesh/Core/System/omstream.cc
        OpenMesh/Core/System/omstream.hh
        OpenMesh/Core/System/OpenMeshDLLMacros.hh
        OpenMesh/Core/Templates/newClass.cc
        OpenMesh/Core/Templates/newClass.hh
        OpenMesh/Core/Templates/newClassT.cc
        OpenMesh/Core/Templates/newClassT.hh
        OpenMesh/Core/Utils/AutoPropertyHandleT.hh
        OpenMesh/Core/Utils/BaseProperty.cc
        OpenMesh/Core/Utils/BaseProperty.hh
        OpenMesh/Core/Utils/color_cast.hh
        OpenMesh/Core/Utils/Endian.cc
        OpenMesh/Core/Utils/Endian.hh
        OpenMesh/Core/Utils/GenProg.hh
        OpenMesh/Core/Utils/Noncopyable.hh
        OpenMesh/Core/Utils/Property.hh
        OpenMesh/Core/Utils/PropertyContainer.hh
        OpenMesh/Core/Utils/PropertyManager.hh
        OpenMesh/Core/Utils/RandomNumberGenerator.cc
        OpenMesh/Core/Utils/RandomNumberGenerator.hh
        OpenMesh/Core/Utils/SingletonT.cc
        OpenMesh/Core/Utils/SingletonT.hh
        OpenMesh/Core/Utils/vector_cast.hh
        OpenMesh/Core/Utils/vector_traits.hh
        denoiser.h
	ptree.cpp
        main.cpp)
