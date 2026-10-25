// dChunkParsel.cpp: implementation of the dChunkParsel class.
//
//////////////////////////////////////////////////////////////////////
#include <stdafx.h>
#include "dTree.h"
#include "dChunkParsel.h"

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////


// file chunk reader
class dChunkHeader
{
	public:

	// read file chunk id and size
	dChunkHeader (FILE* file)
	{
		fread (&m_id, sizeof (dUnsigned32), 1, file);
		fread (&m_size, sizeof (dUnsigned32), 1, file);

		m_id = SWAP_INT32(m_id);
		m_size = SWAP_INT32(m_size);
	}

	// read file chunk id and size, and deduct chunk size
	dChunkHeader ReadBlock (FILE* file) 
	{
		dChunkHeader block (file);
		m_size -= sizeof(dChunkHeader);
		m_size -= block.m_size;
		return block;
	}

	// skip unknown block chunk
	void SkipBlock (FILE* file) 
	{
		fseek (file, m_size, SEEK_CUR);
	}

	dUnsigned32 m_id;
	dUnsigned32 m_size;
};



dChunkParsel::dChunkParsel(FILE *file)
	:dList<dBucket>()    
{
	m_file = file;
	
}

dChunkParsel::~dChunkParsel()
{
}


void dChunkParsel::RegisterChunk (dUnsigned32 chunkID, ChunkProcess fnt, const char* description)
{
	dChunkDescriptor header;
	header.m_readFnction = fnt;
	header.m_description = description;
 	GetLast()->GetInfo().Insert (header, chunkID);
}		   

void dChunkParsel::FlushChunk ()
{
	Remove(GetLast());
}


void dChunkParsel::BeginParse (dUnsigned32 m_chunkID, ChunkProcess fnt, const char* description)
{
	// read the header chunk
	dChunkHeader header (m_file);
	if (header.m_id == m_chunkID) {
		// if the chunk id match the header chunk load the file
		Append();
		(this->*fnt)(&header, description);
		FlushChunk();
	}
}


dInt32 dChunkParsel::ParceChunk(void *handle)
{
	dBucket* bucket;
	dChunkDescriptor* desc;
	dBucket::dTreeNode* node;

	dChunkHeader& header = *((dChunkHeader*) handle);
	while (header.m_size) {
		dChunkHeader block (header.ReadBlock(m_file));
	  	bucket = &GetLast()->GetInfo();
		node = bucket->Find(block.m_id);
		if (node) {
			desc = &node->GetInfo();	
			Append();	
			(this->*desc->m_readFnction)(&block, desc->m_description);
			FlushChunk();
		} else {
			block.SkipBlock (m_file);
		}
	}
  
	return 0;
}


void dChunkParsel::ReadData (void* handle, void* output)
{
	dChunkHeader& block = *((dChunkHeader*) handle);
	fread (output, block.m_size, 1, m_file);
}

