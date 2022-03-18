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
 * grid.h
 *
 *  Created on: 2018/03/15
 *      Author: alexandr <alexander@g.sp.m.is.nagoya-u.ac.jp>
 */

#ifndef GRID_H_
#define GRID_H_

#include "cell.h"
//STD
#include <vector>

namespace ground_mapping {
	class Grid {
		public:
			/**
			 * \brief Constructor
			 */
			Grid():
				cell_size_(0.0),
				width_(0),
				height_(0) {
				grid_.clear();
			}
			/**
			 * \brief Destructor
			 */
			~Grid() {
				;//nothing
			}
			/**
			 * \brief Clears the grid contents
			 */
			void clear() {
				for (auto& cell : grid_) {
					cell.clear();
				}
			}
			/**
			 * \brief Re-sizes the grid with the given parameters
			 * \param[in] width is the horizontal number of cells
			 * \param[in] height is the vertical number of cells
			 * \param[in] cell_size is the resolution (in m) of a single cell
			 */
			void resize(int width, int height, float cell_size) {
				cell_size_ = cell_size;
				width_ = width;
				height_ = height;
				grid_.clear();
				grid_.resize((width_ * height_)/(cell_size_ * cell_size_));
			}
			/**
			 * \brief Adds a new point to the grid cell [u,v]
			 * \param[in] u is the horizontal cell index, where u in <0..width>
			 * \param[in] v is the vertical cell index, where v in <0..heigth>
			 * \param[in] p is the new point to add
			 * \Pre: the grid has been initialised (resize) beforehand
			 */
			void addPoint(int u, int v, pcl::PointXYZ& p) {
				if (u >= 0 && u < width_ && v >= 0 && v < height_) {
					grid_[v*width_ + u].addPoint(p);
				} /*else {
					printf("u,v = %d,%d, w,h = %d,%d\n", u,v,width_,height_);
				}*/
			}
			/**
			 * \brief Overloaded version: accepts PointXYZI
			 */
			void addPoint(int u, int v, pcl::PointXYZI& p) {
				if (u >= 0 && u < width_ && v >= 0 && v < height_) {
					grid_[v*width_ + u].addPoint(p);
				} /*else {
					printf("u,v = %d,%d, w,h = %d,%d\n", u,v,width_,height_);
				}*/
			}
			/**
			 * \brief Overloaded version: <u,v> indexes are obtained from the point
			 */
			void addPoint(pcl::PointXYZ& p) {
				int u = (int)((p.x+width_/2.0)/cell_size_);
				int v = (int)((p.y+height_/2.0)/cell_size_);
				addPoint(u,v,p);
			}
			/**
			 * \brief Overloaded version: accepts PointXYZI
			 */
			void addPoint(pcl::PointXYZI& p) {
				int u = (int)((p.x+width_/2.0)/cell_size_);
				int v = (int)((p.y+height_/2.0)/cell_size_);
				addPoint(u,v,p);
			}
			/**
			 * \brief Updates all the grid cells
			 */
			void update(); //implemented on the CPP file, somehow omp and some C++11 extensions didn't work here
			/**
			 * \brief gets the current grid width
			 */
			int width() const { return width_; }
			/**
			 * \brief gets the current grid height
			 */
			int height() const { return width_; }
			/**
			 * \brief gets the current grid cell size
			 */
			float cellSize() const { return cell_size_; }
			/**
			 * \brief gets the grid's cell by its indexes
			 * \param[in] u is horizontal coord. index
			 * \param[in] v is vertical coord. index
			 */
			Cell cell(int u, int v) const {
				if (u >= 0 && u < width_ && v >= 0 && v < height_) {
					return grid_[v*width_+u];
				} else {
					//printf("cell at (%d,%d) does not exist (limits %d,%d)\n", u, v, width_, height_);
					return Cell();
				}
			}
			/**
			 * \brief gets a reference to the grid's cell by its indexes
			 * \param[in] u is horizontal coord. index
			 * \param[in] v is vertical coord. index
			 */
			Cell& cell(int u, int v) {
				if (u >= 0 && u < width_ && v >= 0 && v < height_) {
					return grid_[v*width_+u];
				} else {
					//printf("cell at (%d,%d) does not exist (limits %d,%d)\n", u, v, width_, height_);
					return null_cell_;
				}
			}
			/**
			 * \brief gets the grid's cell by its indexes
			 * \param[in] u is horizontal coord. index
			 * \param[in] v is vertical coord. index
			 */
			Cell at(int u, int v) const {
				if (u >= 0 && u < width_ && v >= 0 && v < height_) {
					return grid_[v*width_+u];
				} else {
					//printf("cell at (%d,%d) does not exist (limits %d,%d)\n", u, v, width_, height_);
					return Cell();
				}
			}
			/**
			 * \brief gets a reference to the grid's cell by its indexes
			 * \param[in] u is horizontal coord. index
			 * \param[in] v is vertical coord. index
			 */
			Cell& at(int u, int v) {
				if (u >= 0 && u < width_ && v >= 0 && v < height_) {
					return grid_[v*width_+u];
				} else {
					//printf("cell at (%d,%d) does not exist (limits %d,%d)\n", u, v, width_, height_);
					return null_cell_;
				}
			}
			/**
			 * \brief Access all the grid cells
			 */
			std::vector< Cell > grid() const { return grid_; }
			/**
			 * \brief Access all the grid cells as reference
			 */
			std::vector< Cell >& grid() { return grid_; }
			/**
				\brief Output stream operator
			*/
			friend std::ostream& operator<<(std::ostream& out, const Grid& g)
			{
				for (int v = 0; v < g.height(); v++) {
					for (int u = 0; u < g.width(); u++) {
						if (g.cell(u,v).size() > 3) {
							out << g.width() << "," << g.height() << ",";
							out << u << "," << v << "," << g.cell(u,v);
							out << std::endl;
						}
					}
				}

				return out;
			}
			/**
			 * \brief Gets the number of points in the grid
			 */
			size_t size() {
				size_t grid_size = 0;
				for (auto& cell : grid_) {
					grid_size += cell.size();
				}
				return grid_size;
			}
			/**
			 * \brief Applies a transform to the cell's pointcloud
			 */
			void transformPointCloud(const tf::Transform &transform);

		private:
			std::vector< Cell > grid_; //! grid is kept as single vector
			float cell_size_; //! size (in m) of a single cell (cells are square)
			int width_; //! horizontal number of cells in the grid
			int height_; //! vertical number of cells in the grid
			Cell null_cell_;
	};
} //namespace

#endif /* COMPUTING_PERCEPTION_SEMANTICS_PACKAGES_LIDAR_ROAD_SURFACE_NODES_GROUND_MAPPER_GRID_H_ */
