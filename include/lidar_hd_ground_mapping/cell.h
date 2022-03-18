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
 * cell.h
 *
 *  Created on: 2018/03/15
 *      Author: alexandr <alexander@g.sp.m.is.nagoya-u.ac.jp>
 */

#ifndef CELL_H_
#define CELL_H_

//STD
#include <limits>
#include <iostream>
//Eigen
#include <Eigen/Core>
#include <Eigen/Dense>
//PCL-ROS
#include <pcl_ros/point_cloud.h>
#include <pcl/common/centroid.h>
#include <pcl/common/eigen.h>
#include <pcl/features/feature.h>
#include <pcl/features/normal_3d.h>
#include <pcl_ros/transforms.h>

namespace ground_mapping {
	class Cell {
		private:
		    pcl::PointCloud<pcl::PointXYZ> points_;
		    Eigen::Vector4f centroid_;
			Eigen::Matrix3f covariance_;
			float curvature_;
			Eigen::Vector3f eigen_values_;
			Eigen::Matrix3f eigen_vectors_;
			Eigen::Vector4f plane_parameters_;

		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		public:
		    /**
		     * \brief Constructor
		     */
			Cell() {
				clear();
			}
		    /**
		     * \brief Destructor
		     */
			~Cell() {
				; //nothing
			}

		    /**
		     * \brief Adds a new point to the cell
		     * \param[in] p is a new point to add
		     */
			void addPoint(pcl::PointXYZ p) {
				points_.push_back(p);
			}
			/**
			 * \brief Overloaded version: accepts PointXYZI
			 * \note: discards intensity value
			 */
			void addPoint(pcl::PointXYZI p) {
				pcl::PointXYZ pt(p.x,p.y,p.z);
				points_.push_back(pt);
			}
		    /**
		     * \brief Clears all data from this cell
		     */
			void clear() {
				points_.clear();
				centroid_.setConstant(0);
				covariance_.setConstant(0);
				eigen_values_.setConstant(0);
				eigen_vectors_.setConstant(0);
				plane_parameters_.setConstant (std::numeric_limits<float>::quiet_NaN ());
				curvature_ = std::numeric_limits<float>::quiet_NaN ();
			}
		    /**
		     * \brief Updates this cell's statistics from the list of points
		     */
			void update() {
				//if too small, return
				if (points_.size() < 3) {
					return;
				}
				//gets the average point (centroid) and covariance matrix
				if (!(pcl::computeMeanAndCovarianceMatrix(points_, covariance_, centroid_))) {
					return;
				}
				//gets the eigenvalues
				pcl::eigen33(covariance_, eigen_vectors_, eigen_values_);
				//sort the Eigen values
				std::sort(eigen_values_.data(), eigen_values_.data() + eigen_values_.size());

				//gets the plane normal and surface curvature
				pcl::solvePlaneParameters(covariance_, centroid_, plane_parameters_, curvature_);
			}
		    /**
		     * \brief Tells if this cell is part of the ground (true) or not (false)
		     * using PCA
		     */
			bool isGround(bool do_filter=false) {
				if (!do_filter) {
					return true;
				} else {
					//Idea taken from:
					//Li et al., "An Improved RANSAC for 3D Point Cloud Plane Segmentation Based on Normal Distribution Transformation Cells",
					//Remote Sens. 2017, 9(5), 433; doi:10.3390/rs9050433
					//Given eigen values $\lambda_1 \le \lambda_2 \le \lambda_3$, the following relations exist:
					//$\frac{\lambda_2}{\lambda_3} \le te$, the distribution is linear
					//not linear and $\frac{\lambda_1}{\lambda_2} \le te$, the distribution is planar
					//no eigen value $\lambda_i$ is 1/te times larger than another one, the distribution is spherical
					const double te = 0.04;

					return (points_.size() >= 3 //big enough
							&& (eigen_values_[1]/eigen_values_[2]) > te //and not linear
 							&& (eigen_values_[0]/eigen_values_[1]) <= te
							); //and planar
				}
			}
			/**
				\brief Output stream operator
			*/
			friend std::ostream& operator<<(std::ostream& out, const Cell& cell) {
				Eigen::IOFormat OctaveFmt(Eigen::StreamPrecision, 0, ",", ",", "", "", "", "");
				out << cell.size() << ",";
				out << cell.centroid_.format(OctaveFmt) << ",";
				out << cell.covariance_.format(OctaveFmt) << ",";
				out << cell.eigen_values_.format(OctaveFmt) << ",";
				out << cell.eigen_vectors_.format(OctaveFmt) << ",";
				out << cell.plane_parameters_.format(OctaveFmt) << ",";
				out << cell.curvature_;
				for (auto p : cell.points_.points) {
					out << "," << p.x << "," << p.y << "," << p.z;
				}

				return out;
			}
			/**
			 * \brief Gets the number of points in the cell
			 */
			size_t size() const { return points_.points.size(); }
			/**
			 * \brief Gets the cell's points
			 */
			pcl::PointCloud<pcl::PointXYZ> points() const { return points_; }
			/**
			 * \brief Gets the cell's points as reference
			 */
			pcl::PointCloud<pcl::PointXYZ>& points() { return points_; }
			/**
			 * \brief Get the average (centroid) point
			 */
			void centroid(Eigen::Vector3f &c) {
				c << centroid_[0],centroid_[1],centroid_[2];
			}
			/**
			 * \brief Overloaded version: returns a point
			 */
			pcl::PointXYZ centroid() {
				return pcl::PointXYZ(centroid_[0], centroid_[1], centroid_[2]);
			}
			/**
			 * \brief Gets the covariance matrix
			 */
			Eigen::Matrix3f covariance() const { return covariance_; }
			/**
			 * \brief Get the eigen values
			 */
			Eigen::Vector3f eigenValues() const { return eigen_values_; }
			/**
			 * \brief Gets the eigen vectors matrix
			 */
			Eigen::Matrix3f eigenVectors() const { return eigen_vectors_; }
			/**
			 * \brief Get the plane parameters A,B,C,D
			 */
			Eigen::Vector4f planeParams() const { return plane_parameters_; }
			/**
			 * \brief Get the plane parameters A,B,C,D
			 */
			void planeParams(float &A, float &B, float &C, float &D) const {
				A = plane_parameters_[0];
				B = plane_parameters_[1];
				C = plane_parameters_[2];
				D = plane_parameters_[3];
			}
			/**
			 * \brief Get the plane normal vector
			 */
			void normalVector(Eigen::Vector3f &n) {
				n << plane_parameters_[0], plane_parameters_[1], plane_parameters_[2];
			}
			/**
			 * \brief Overloaded version
			 */
			pcl::PointXYZ normalVector() {
				Eigen::Vector3f n;
				n << plane_parameters_[0], plane_parameters_[1], plane_parameters_[2];
				return pcl::PointXYZ(n[0], n[1], n[2]);
			}
			/**
			 * \brief Gets the curvature value
			 */
			float curvature() const { return curvature_; }
			/**
			 * \brief Applies a transform to the cell's pointcloud
			 */
			void transformPointCloud(const tf::Transform &transform) {
				pcl_ros::transformPointCloud(points_, points_, transform);
			}
	};
} //namespace


#endif /* CELL_H_ */
