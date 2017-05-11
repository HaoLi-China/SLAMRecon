// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM
#ifndef _FE_MESHINGENGINE_CUDA_H
#define _FE_MESHINGENGINE_CUDA_H

#include "../FEMeshingEngine.h"

namespace FE
{
	template<class TVoxel, class TIndex>
	class FEMeshingEngine_CUDA : public FEMeshingEngine < TVoxel, TIndex >
	{};

	template<class TVoxel>
	class FEMeshingEngine_CUDA<TVoxel, FEVoxelBlockHash> : public FEMeshingEngine < TVoxel, FEVoxelBlockHash >
	{
	private:
		unsigned int  *noTriangles_device;
		Vector4s *visibleBlockGlobalPos_device;

	public:
		void MeshScene(FEMesh *mesh, const FEScene<TVoxel, FEVoxelBlockHash> *scene);

		FEMeshingEngine_CUDA(void);
		~FEMeshingEngine_CUDA(void);
	};
}
#endif //_FE_MESHINGENGINE_CUDA_H
