#include "C3dParser.h"
	
C3dParser::~C3dParser(void)
{
	input_file_stream.close();
}
	
void C3dParser::OpenFile(const char* fn)
{
	input_file_stream.open(fn, std::ios::in | std::ios::binary);
	
	if (input_file_stream.is_open() == false) {
		printf("[C3dParser] Error opening c3d file.\n");
		return;
	} else {
		printf("[C3dParser] C3d file open.\n");
	}

	ReadHeader();
	ReadParameterSections();
}

void C3dParser::ReadHeader(void)
{
	char header_block[512];

	input_file_stream.read(header_block, 512);

	if (header_block[1] == 0x50 && header_block[0] == 0x02) {
		printf("[C3dParser] Header detected\n");
	} else {
		return;
	}
	
	int start_frame = (boost::int16_t)(((unsigned char)header_block[6]) + (((unsigned char)header_block[7]) << 8));
	int end_frame = (boost::int16_t)((unsigned char)header_block[8] + (((unsigned char)header_block[9]) << 8));
	
	no_frames_ = end_frame-start_frame+1;
	printf("[C3dParser] Number of samples: %d\n", no_frames_);
	
	no_markers_ = (boost::int16_t)((unsigned char)header_block[2] + (((unsigned char)header_block[3]) << 8));
	printf("[C3dParser] Number of markers: %d\n", no_markers_);	
}

void C3dParser::ReadParameterSections(void)
{	
	char parameter_block[4096];

	input_file_stream.read(parameter_block, 4);
	
	int no_parameter_blocks = parameter_block[2];
	printf("[C3dParser] No. parameter blocks: %d\n", no_parameter_blocks);
	
	input_file_stream.read(parameter_block, 512*4-4); // ((no_parameter_blocks-1)*512)-4);
	
	int group_start = 0;
	bool end_of_block = false;
	while (end_of_block == false) {
		int id = parameter_block[group_start+1];
		if (id < 0) {
			printf("GROUP ID: %d\n", id);
			
			int no_group_chars = abs(parameter_block[group_start]);			
			printf("Group name: ");
			for (int i=0; i < no_group_chars; ++i) {
				printf("%c", parameter_block[group_start+2+i]);
			}
			printf("\n");
			
			int offset_to_next = parameter_block[group_start+2+no_group_chars] + (parameter_block[group_start+3+no_group_chars] << 8);
			int no_description_chars = parameter_block[group_start+4+no_group_chars];
			printf("Group description: ");
			for (int i=0; i < no_description_chars; ++i) {
				printf("%c", parameter_block[group_start+5+no_group_chars+i]);
			}
			printf("\n");
				
			group_start = group_start+5+no_group_chars+no_description_chars;
		} else {
			printf("PARAMETER ID: %d\n", id);
			
			int no_group_chars = parameter_block[group_start];	
			if (no_group_chars < 0) printf("locked\n");
			printf("Parameter name: ");
			for (int i=0; i < abs(no_group_chars); ++i) {
				printf("%c", parameter_block[group_start+2+i]);
			}
			printf("\n");
			
			int offset_to_next = parameter_block[group_start+2+no_group_chars] + (parameter_block[group_start+3+no_group_chars] << 8);
			printf("Offset: %d\n", offset_to_next);
			if (offset_to_next == 0) {
				end_of_block = true;
			}
			
			int data_length = parameter_block[group_start+4+no_group_chars];
			printf("Data length: %d\n", data_length);
			
			int no_dimensions = parameter_block[group_start+5+no_group_chars];
			printf("No. Dimensions: %d\n", no_dimensions);			
			
			if (no_dimensions > 0) {
				int dim_size[8];
				int data_offset;
				
				for (int i=0; i<no_dimensions; ++i) {
					dim_size[i] = parameter_block[group_start+6+no_group_chars+i];
					printf("Dimension %d size: %d\n", i, dim_size[i]);	
				}
			
				if (data_length == -1 && no_dimensions == 2) {
					for (int k=0; k<dim_size[1]; ++k) {
						printf("Parameter value: ");
						for (int i=0; i<dim_size[0]; ++i) {
							printf("%c", parameter_block[group_start+6+no_dimensions+no_group_chars+i+dim_size[0]*k]);
						}
						printf("\n");
					}
					data_offset = dim_size[0] * dim_size[1];
				} else if (data_length == -1 && no_dimensions == 1) {
					printf("Parameter value: ");
					for (int i=0; i<dim_size[0]; ++i) {
						printf("%c", parameter_block[group_start+6+no_dimensions+no_group_chars+i]);
					}
					printf("\n");
					
					data_offset = dim_size[0];
				} else {
					printf("Can't parse parameter data\n");
					return;
				}
				
				int no_description_chars = parameter_block[group_start+6+no_dimensions+no_group_chars+data_offset];
				printf("Parameter description: ");
				for (int i=0; i < no_description_chars; ++i) {
					printf("%c", parameter_block[group_start+7+no_dimensions+no_group_chars+i+data_offset]);
				}
				printf("\n");
				
				group_start = group_start+7+no_dimensions+no_group_chars+no_description_chars+data_offset;
			} else {			
				int parameter_value = parameter_block[group_start+6+no_group_chars] + (parameter_block[group_start+7+no_group_chars] << 8);
				printf("Parameter value: %d\n", parameter_value);			
				
				int no_description_chars = parameter_block[group_start+6+no_group_chars+data_length];
				printf("Parameter description: ");
				for (int i=0; i < no_description_chars; ++i) {
					printf("%c", parameter_block[group_start+7+no_group_chars+i+data_length]);
				}
				printf("\n");
				
				group_start = group_start+7+no_group_chars+no_description_chars+data_length;
			}
		}
	}
}

#include "windows.h"

bool C3dParser::ReadData(double *data_x, double *data_y, double *data_z)
{
	char data_block[64];
	
	input_file_stream.read(data_block, 8);
			
	if (input_file_stream.good() == false) return false;
		
	int x_int = (boost::int16_t)(((unsigned char) data_block[1]) << 8 | (unsigned char) data_block[0]);
	int y_int = (boost::int16_t)(((unsigned char) data_block[3]) << 8 | (unsigned char) data_block[2]);
	int z_int = (boost::int16_t)(((unsigned char) data_block[5]) << 8 | (unsigned char) data_block[4]);
		
	*data_x = ((double) x_int) * 0.1449;
	*data_y = ((double) y_int) * 0.1449;
	*data_z = ((double) z_int) * 0.1449;
	
	return true;
}

int C3dParser::GetNoFrames(void) 
{
	return no_frames_;
}

int C3dParser::GetNoMarkers(void) 
{
	return no_markers_;
}