// %Tag(FULLTEXT)%
/*
 *  Copyright (c) 2018, Nagoya University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 *  * Neither the name of Autoware nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 *  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
/*
 * pfm2pgm.cpp
 *
 *  Created on: 2018/07/16
 *      Author: alexandr <alexander@g.sp.m.is.nagoya-u.ac.jp>
 */

#include <iostream>
#include <cstdio>
#include <cstdlib>
#include <string>

const double HMAX=200.0;
const double HMIN=-800.0;
class DepthImage {
	public:
	DepthImage()
	{
		width_ = 0;
		height_ = 0;
		buffer_ = (float*)NULL;
		center_z_ = 0.0;
	};

	~DepthImage()
	{
		if (buffer_) {
			delete buffer_;
		}
	};

	void setCenterZ(const float z)
	{
		center_z_ = z;
	}

	void loadDepthImage(const std::string filename)
	{
		//try reading depth image file
		std::string target_filename = filename;
		FILE *fp;
		fp=fopen(target_filename.c_str(),"r");
		if (!fp) {
			printf("Failed opening \"%s\" file for reading\n", target_filename.c_str());
			return;
		}
		width_ = 0;
		height_ = 0;
		int nelem = fscanf(fp,"PF\n%d %d\n1.0\n", &width_, &height_);
		if (nelem < 2 || width_ <= 0 || height_ <= 0) {
			printf("Image dimensions (%d,%d) are invalid.\n", width_, height_);
			fclose(fp);
			return;
		}

		if (width_*height_ <= 0) {
			return;
		}
		buffer_ = new float[width_*height_];
		size_t count = 0;
		count = fread(buffer_, sizeof(float), width_*height_, fp);
		fclose(fp);

		if (count < width_*height_) {
			printf("Failed reading image data. Read %d elements, expected %d\n", (int)count, width_*height_);
			return;
		}
	}

	void saveDepthImage(const std::string filename, bool asPFM)
	{
		FILE *fp;
		FILE *fp_txt;

		if (!buffer_) {
			return;
		}

		fp=fopen(filename.c_str(),"w");
		fp_txt=fopen((filename+".txt").c_str(),"w");

		if (asPFM) {
			fprintf(fp,"PF\n%d %d\n1.0\n", width_, height_);
			fwrite(buffer_, sizeof(float), width_*height_, fp);
		} else {
			float min_val = -1.0; //1.0e8;
			float max_val = 1.0; //-1.0e8;
			size_t j = 0;
			for (size_t i = 0; i < width_*height_; i++) {
				fprintf(fp_txt, "%f", buffer_[i]);
                j++;
				if (j >= width_) {
				    fprintf(fp_txt, "\n");
                    j = 0;
				} else {
				    fprintf(fp_txt, ",");
				}
			}

			//reverse the glortho transform for height
			//const float tz = -1.0 * (HMAX+HMIN)/(HMAX-HMIN);
			//const float fz = 1.0/(-2.0/(HMAX-HMIN));
			//for (size_t i = 0; i < width_*height_; i++) {
			//	buffer_[i] = (buffer_[i] - tz)*fz;
			//}
			const float tz = (HMAX-HMIN);
			for (size_t i = 0; i < width_*height_; i++) {
				buffer_[i] = (buffer_[i]*tz) + HMIN + -center_z_;
			}

//			//find min/max values
//			for (size_t i = 0; i < width_*height_; i++) {
//				if (buffer_[i] > -1 && buffer_[i] < 1) {
//					if (buffer_[i] > max_val) {
//						max_val = buffer_[i];
//					}
//					if (buffer_[i] < min_val) {
//						min_val = buffer_[i];
//					}
//				}
//			}
//			std::cout << "Max=" << max_val << " min=" << min_val << std::endl;
			//save the PGM file (16 bits)
			const float max_scale = 65535.0;
			const float epsilon = 1.0e-6;
			float value = 0.0;
			unsigned short int valInt = 0;
			fprintf(fp,"P5\n%d %d\n%d\n", width_, height_, (unsigned short int)max_scale);
			for (size_t i = 0; i < width_*height_; i++) {
				if (buffer_[i] <= -1.0 || buffer_[i] >= 1.0) {
					value = 0.0;
				} else {
					value = max_scale - (((buffer_[i] - min_val))/(max_val - min_val))*max_scale;
				}
				valInt = (unsigned short int)value;
				valInt = (valInt & 0xFF)<<8 | ((valInt >> 8) & 0xFF);
				//fwrite((char*)&valInt+1, 1, 1, fp);
				//fwrite((char*)&valInt, 1, 1, fp);
				fwrite((char*)&valInt, 2, 1, fp);
			}

		}
		fclose(fp);
		fclose(fp_txt);
	}

	private:
		float* buffer_;
		size_t width_;
		size_t height_;
		float center_z_;
};

int main(int argc, char **argv)
{
	DepthImage depthImage;

	if (argc <= 1) {
		std::cout << "pfm2pgm: convert pfm maps to pgm image format" << std::endl;
		std::cout << "Please provide as argument pfm filename to convert" << std::endl;
		exit(1);
	}

	std::string filename = std::string(argv[1]);
	std::string new_file = filename.substr(0, filename.find_last_of('.')) + ".pgm";
	std::cout << "Convert depth image \"" << filename << "\" into image file \"" << new_file << "\"" << std::endl;

	if (argc > 2) {
		depthImage.setCenterZ(std::stof(std::string(argv[2])));
	}

	depthImage.loadDepthImage(filename);
	depthImage.saveDepthImage(new_file,false);


    return 0;
}
