// dChunkParsel.h: interface for the dChunkParsel class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_DCHUNKPARSEL_H__9136D67A_D7DF_455D_B281_A430D389C265__INCLUDED_)
#define AFX_DCHUNKPARSEL_H__9136D67A_D7DF_455D_B281_A430D389C265__INCLUDED_

#include <stdafx.h>
#include "dTree.h"


class dChunkParsel;
typedef dInt32 (dChunkParsel::*ChunkProcess) (void *handle, const char* label);


struct dChunkDescriptor
{
	ChunkProcess  m_readFnction;
	const char* m_description;
};

class dBucket: public dTree<dChunkDescriptor, dUnsigned32>   
{
	public: 
	dBucket ()
		:dTree<dChunkDescriptor, dUnsigned32>()   
	{
	}
};

class dChunkParsel: public dList<dBucket>  
{
	public:
	dChunkParsel(FILE * file);
	virtual ~dChunkParsel();


	void ReadData (void* handle, void* output);

	void BeginParse (dUnsigned32 m_chunkID, ChunkProcess fnt, const char* description); 

	protected:
	void RegisterChunk (dUnsigned32 m_chunkID, ChunkProcess fnt, const char* description); 
	dInt32 ParceChunk(void* handle);
	void FlushChunk ();

	FILE *m_file;

};

#endif 
