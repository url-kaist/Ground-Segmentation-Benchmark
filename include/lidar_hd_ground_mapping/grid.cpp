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
 * grid.cpp
 *
 *  Created on: 2018/03/15
 *      Author: alexandr <alexander@g.sp.m.is.nagoya-u.ac.jp>
 */

#include "grid.h"
#include "cell.h"
//STD
#include <iostream>
#include <chrono> //for testing
//OMP
#include <omp.h>

namespace ground_mapping {
	void Grid::update()
	{
		if (!(grid_.size())) {
			return;
		}
		auto start = std::chrono::high_resolution_clock::now();
		#pragma omp parallel for default(shared)
		//for (auto& cell : grid_) { //OMP doesn't work with C++11 for auto syntax
		for (std::vector< Cell >::iterator it = grid_.begin(); it < grid_.end(); ++it) {
			it->update();
		}
		auto elapsed = std::chrono::high_resolution_clock::now() - start;
		long long microseconds = std::chrono::duration_cast<std::chrono::microseconds>(elapsed).count();
		std::cout << "Grid::update() computed in " <<(double)microseconds/1000000.0 << " seconds" <<  std::endl;
	}

	void Grid::transformPointCloud(const tf::Transform &transform)
	{
		if (!(grid_.size())) {
			return;
		}
		auto start = std::chrono::high_resolution_clock::now();
		#pragma omp parallel for default(shared)
		for (std::vector< Cell >::iterator it = grid_.begin(); it < grid_.end(); ++it) {
			it->transformPointCloud(transform);
		}
		auto elapsed = std::chrono::high_resolution_clock::now() - start;
		long long microseconds = std::chrono::duration_cast<std::chrono::microseconds>(elapsed).count();
		std::cout << "Grid::transformPointCloud() computed in " <<(double)microseconds/1000000.0 << " seconds" <<  std::endl;
	}
} //namespace
