// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM
#ifndef _FE_MESH_H
#define _FE_MESH_H

#include "../Utils/FELibDefines.h"
#include "Image.h"

#include <stdlib.h>

namespace FE
{
	class FEMesh
	{
	public:
		struct Triangle { Vector3f p0, p1, p2; };

		MemoryDeviceType memoryType;

		uint noTotalTriangles;
		static const uint noMaxTriangles = SDF_LOCAL_BLOCK_NUM * 32;

		Basis::MemoryBlock<Triangle> *triangles;

		explicit FEMesh(MemoryDeviceType memoryType)
		{
			this->memoryType = memoryType;
			this->noTotalTriangles = 0;

			triangles = new Basis::MemoryBlock<Triangle>(noMaxTriangles, memoryType);
		}

		void WriteOBJ(const char *fileName)
		{
			Basis::MemoryBlock<Triangle> *cpu_triangles; bool shoulDelete = false;
			if (memoryType == MEMORYDEVICE_CUDA)
			{
				cpu_triangles = new Basis::MemoryBlock<Triangle>(noMaxTriangles, MEMORYDEVICE_CPU);
				cpu_triangles->SetFrom(triangles, Basis::MemoryBlock<Triangle>::CUDA_TO_CPU);
				shoulDelete = true;
			}
			else cpu_triangles = triangles;

			Triangle *triangleArray = cpu_triangles->GetData(MEMORYDEVICE_CPU);

			FILE *f = fopen(fileName, "w+");
			if (f != NULL)
			{
				for (uint i = 0; i < noTotalTriangles; i++)
				{
					fprintf(f, "v %f %f %f\n", triangleArray[i].p0.x, triangleArray[i].p0.y, triangleArray[i].p0.z);
					fprintf(f, "v %f %f %f\n", triangleArray[i].p1.x, triangleArray[i].p1.y, triangleArray[i].p1.z);
					fprintf(f, "v %f %f %f\n", triangleArray[i].p2.x, triangleArray[i].p2.y, triangleArray[i].p2.z);
				}

				for (uint i = 0; i < noTotalTriangles; i++) fprintf(f, "f %d %d %d\n", i * 3 + 2 + 1, i * 3 + 1 + 1, i * 3 + 0 + 1);
				fclose(f);
			}

			if (shoulDelete) delete cpu_triangles;
		}

		void WriteSTL(const char *fileName)
		{
			Basis::MemoryBlock<Triangle> *cpu_triangles; bool shoulDelete = false;
			if (memoryType == MEMORYDEVICE_CUDA)
			{
				cpu_triangles = new Basis::MemoryBlock<Triangle>(noMaxTriangles, MEMORYDEVICE_CPU);
				cpu_triangles->SetFrom(triangles, Basis::MemoryBlock<Triangle>::CUDA_TO_CPU);
				shoulDelete = true;
			}
			else cpu_triangles = triangles;

			Triangle *triangleArray = cpu_triangles->GetData(MEMORYDEVICE_CPU);

			FILE *f = fopen(fileName, "wb+");

			if (f != NULL) {
				for (int i = 0; i < 80; i++) fwrite(" ", sizeof(char), 1, f);

				fwrite(&noTotalTriangles, sizeof(int), 1, f);

				float zero = 0.0f; short attribute = 0;
				for (uint i = 0; i < noTotalTriangles; i++)
				{
					fwrite(&zero, sizeof(float), 1, f); fwrite(&zero, sizeof(float), 1, f); fwrite(&zero, sizeof(float), 1, f);

					fwrite(&triangleArray[i].p2.x, sizeof(float), 1, f);
					fwrite(&triangleArray[i].p2.y, sizeof(float), 1, f);
					fwrite(&triangleArray[i].p2.z, sizeof(float), 1, f);

					fwrite(&triangleArray[i].p1.x, sizeof(float), 1, f);
					fwrite(&triangleArray[i].p1.y, sizeof(float), 1, f);
					fwrite(&triangleArray[i].p1.z, sizeof(float), 1, f);

					fwrite(&triangleArray[i].p0.x, sizeof(float), 1, f);
					fwrite(&triangleArray[i].p0.y, sizeof(float), 1, f);
					fwrite(&triangleArray[i].p0.z, sizeof(float), 1, f);

					fwrite(&attribute, sizeof(short), 1, f);

					//fprintf(f, "v %f %f %f\n", triangleArray[i].p0.x, triangleArray[i].p0.y, triangleArray[i].p0.z);
					//fprintf(f, "v %f %f %f\n", triangleArray[i].p1.x, triangleArray[i].p1.y, triangleArray[i].p1.z);
					//fprintf(f, "v %f %f %f\n", triangleArray[i].p2.x, triangleArray[i].p2.y, triangleArray[i].p2.z);
				}

				//for (uint i = 0; i<noTotalTriangles; i++) fprintf(f, "f %d %d %d\n", i * 3 + 2 + 1, i * 3 + 1 + 1, i * 3 + 0 + 1);
				fclose(f);
			}

			if (shoulDelete) delete cpu_triangles;
		}

		~FEMesh()
		{
			delete triangles;
		}

		// Suppress the default copy constructor and assignment operator
		FEMesh(const FEMesh&);
		FEMesh& operator=(const FEMesh&);
	};
}
#endif //_FE_MESH_H
