// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM.
#ifndef _FE_MESHINGENGINE_H
#define _FE_MESHINGENGINE_H

#include <math.h>

#include "../Utils/FELibDefines.h"

#include "../Objects/FEScene.h"
#include "../Objects/FEMesh.h"

namespace FE
{
	template<class TVoxel, class TIndex>
	class FEMeshingEngine
	{
	public:
		virtual void MeshScene(FEMesh *mesh, const FEScene<TVoxel, TIndex> *scene) = 0;

		FEMeshingEngine(void) { }
		virtual ~FEMeshingEngine(void) { }
	};
}
#endif //_FE_MESHINGENGINE_H
