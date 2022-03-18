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
 * point.h
 *
 *  Created on: 2017/09/29
 *      Author: alexandr <alexander@g.sp.m.is.nagoya-u.ac.jp>
 */

#ifndef POINT_H_
#define POINT_H_

//STD
#include <cmath>
#include <vector>
#include <iostream>
#include <limits>
#include <type_traits>
//Eigen
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
//Boost
#include <boost/utility/enable_if.hpp>

namespace ground_mapping {
	/**
		\brief Implements a single 3D point
	*/
	template<typename DataType, int dims = 3>
	class Point {
		private:
			//the actual coordinates
			typedef Eigen::Matrix<DataType,dims,1> Vector_t;
			enum { NeedsToAlign = (sizeof(Vector_t)%16)==0 };
			Vector_t data_;

			EIGEN_MAKE_ALIGNED_OPERATOR_NEW_IF(NeedsToAlign)
		public:
			/**
				\brief Default <0,0,0> point
			*/
			Point()
			{
				clear();
			}
			/**
				\brief Point at <x,y>
			*/
			// template < class T = DataType,
			//            typename boost::enable_if_c< dims == 2, T >::type = 0>
			Point(DataType x, DataType y)
			{
				init(x,y);
				static_assert(dims == 2, "XY constructor only usable in 2D");
			}
			/**
				\brief Point at <x,y,z>
			*/
			// template < class T = DataType,
			//            typename boost::enable_if_c< dims == 3, T >::type = 0>
			Point(DataType x, DataType y, DataType z)
			{
				init(x,y,z);
				static_assert(dims == 3, "XYZ constructor only usable in 3D");
			}
			/**
				\brief Copy constructor
				\param[in] other is the point to copy from
			*/
			Point(const Point<DataType,dims>& other)
			{
				data_ = other.data_;
			}
			/**
				\brief Copy constructor
				\param[in] other is the point to copy from
			*/
			template <typename Y, int dimensions = dims>
			Point(const Point<Y,dimensions>& other)
			{
				static_assert(dimensions == dims, "Point dimensions must be equal");
				for (int i=0; i<data_.rows(); i++) {
					data_(i) = static_cast<DataType>(other.data_(i));
				}
			}

			/**
				\brief Copy operator
				\param[in] other is the point to copy from
			*/
			Point<DataType,dims>& operator=(const Point<DataType,dims>& other)
			{
				// check for self-assignment
				if(&other != this) {
					data_ = other.data_;
				}
				return *this;
			}
			/**
				\brief Copy operator from other point class
				\param[in] other is the point to copy from
			*/
			template <typename Y, int dimensions = dims>
			Point<DataType,dims>& operator=(const Point<Y,dimensions>& other)
			{
				for (int i=0; i<data_.rows(); i++) {
					data_(i) = static_cast<DataType>(other.data_(i));
				}
				return *this;
			}
			/**
				\brief Sum operator
				\param[in] other is the point to add to
			*/
			Point<DataType,dims>& operator+=(const Point<DataType,dims>& other)
			{
				data_ += other.data_;
				return *this;
			}
			/**
				\brief Subtract operator
				\param[in] other is the point to subtract from
			*/
			Point<DataType,dims>& operator-=(const Point<DataType,dims>& other)
			{
				data_ -= other.data_;
				return *this;
			}
			/**
				\brief Sum operator
				\param[in] other is the point to add to
			*/
			friend Point<DataType,dims> operator+(Point<DataType,dims> lhs, const Point<DataType,dims> rhs)
			{
				lhs += rhs;
				return lhs;
			}
			/**
				\brief Subtract operator
				\param[in] other is the point to subtract from
			*/
			friend Point<DataType,dims> operator-(Point<DataType,dims> lhs, const Point<DataType,dims> rhs)
			{
				lhs -= rhs;
				return lhs;
			}
			/**
				\brief Scalar multiplication and assigment
			*/
			Point<DataType,dims>& operator*=(const DataType& s) {
				data_ *= s;
				return *this;
			}
			/**
				\brief Scalar multiplication
				\retval the product of the point p and scalar s
			*/
			friend Point<DataType,dims> operator*(Point<DataType,dims> p, DataType s)
			{
				p *= s;
				return p;
			}
			/**
				\brief Dot product
				\retval the product of this point and the other point
			*/
			DataType operator*(const Point<DataType,dims>& other) const
			{
				return data_.dot(other.data_);
			}
			/**
				\brief Vector inner product and assigment
			*/
			Point<DataType,dims>& operator%=(const Point<DataType,dims>& rhs) {
				data_.array() *= rhs.data_.array();
				return *this;
			}
			/**
				\brief Vector inner product
				\retval the .* (inner product) of this point and the other point
			*/
			friend Point<DataType,dims> operator%(Point<DataType,dims> lhs, const Point<DataType,dims> rhs)
			{
				lhs %= rhs;
				return lhs;
			}
			/**
				\brief Scalar division and assigment
			*/
			Point<DataType,dims>& operator/=(const DataType& s) {
				data_ /= s;
				return *this;
			}
			/**
				\brief Scalar division
				\retval the quotient of lhs point and scalar s
			*/
			friend Point<DataType,dims> operator/(Point<DataType,dims> lhs, DataType s)
			{
				lhs /= s;
				return lhs;
			}
			/**
				\brief Vector inner division and assignment
				\retval the ./ (inner division) of this point and the other point
			*/
			Point<DataType,dims>& operator/=(const Point<DataType,dims>& rhs) {
				data_.array() /= rhs.data_.array();
				return *this;
			}
			/**
				\brief Vector point division
				\retval the ./ (division) of lhs point and rhs point
			*/
			friend Point<DataType,dims> operator/(Point<DataType,dims> lhs, const Point<DataType,dims> rhs)
			{
				lhs /= rhs;
				return lhs;
			}

			/**
				\brief Vector cross product and assignment
				\retval the ./ (inner division) of this point and the other point
			*/
			Point<DataType,dims>& operator*=(const Point<DataType,dims>& rhs) {
				data_ *= rhs.data_;
				return *this;
			}
			/**
				\brief Cross-product
				\retval the cross product of lhs point and rhs point
			*/
			friend Point<DataType,dims> operator*(Point<DataType,dims> lhs, const Point<DataType,dims> rhs)
			{
				lhs *= rhs;
				return lhs;
			}
			/**
				\brief Equal comparison
				\retval true if both points are equal
			*/
			friend bool operator==(const Point<DataType,dims>& lhs, const Point<DataType,dims>& rhs)
			{
				return (lhs.data_.isApprox(rhs.data_));
			}
			/**
				\brief Different comparison
				\retval true if both points are different
			*/
			friend bool operator!=(const Point<DataType,dims>& lhs, const Point<DataType,dims>& rhs)
			{
				return !(lhs == rhs);
			}
			/**
				\brief Less-than comparison
				\retval true if this point is less than the other point
			*/
			friend bool operator<(const Point<DataType,dims>& lhs, const Point<DataType,dims>& rhs)
			{
				return (lhs.data_.array() < rhs.data_.array()).any();
			}
			/**
				\brief Less-than or equal to comparison
				\retval true if this point is less than or equal to the other point
			*/
			friend bool operator<=(const Point<DataType,dims>& lhs, const Point<DataType,dims>& rhs)
			{
				return !(lhs > rhs);
			}
			/**
				\brief Greater-than comparison
				\retval true if this point is greater than the other point
			*/
			friend bool operator>(const Point<DataType,dims>& lhs, const Point<DataType,dims>& rhs)
			{
				return (rhs < lhs);
			}
			/**
				\brief Greater-than or equal to comparison
				\retval true if this point is greater than or equal to the other point
			*/
			friend bool operator>=(const Point<DataType,dims>& lhs, const Point<DataType,dims>& rhs)
			{
				return !(lhs < rhs);
			}
			/**
				\brief Sets the X coordinate of this point
				\param[in] x is the new value
			*/
			inline void setX(const DataType x)
			{
				data_[0] = x;
			}
			/**
				\brief Sets the Y coordinate of this point
				\param[in] y is the new value
			*/
			inline void setY(const DataType y)
			{
				data_[1] = y;
			}
			/**
				\brief Sets the Z coordinate of this point
				\param[in] z is the new value
			*/
			inline void setZ(const DataType z)
			{
				data_[2] = z;
			}
			/**
				\brief X accessor
				\retval the current value of x coordinate
			*/
			DataType x() const
			{
				return data_[0];
			}
			/**
				\brief Y accessor
				\retval the current value of y coordinate
			*/
			DataType y() const
			{
				return data_[1];
			}
			/**
				\brief Z accessor
				\retval the current value of Z coordinate
			*/
			DataType z() const
			{
				return data_[2];
			}
			/**
				\brief X accessor
				\retval the current value of x coordinate
			*/
			DataType& x()
			{
				return data_[0];
			}
			/**
				\brief Y accessor
				\retval the current value of y coordinate
			*/
			DataType& y()
			{
				return data_[1];
			}
			/**
				\brief Z accessor
				\retval the current value of Z coordinate
			*/
			DataType& z()
			{
				return data_[2];
			}
			/**
				\brief Initializer
			*/
			void clear()
			{
				data_.setConstant(0);
			}
			void init(DataType x, DataType y)
			{
				data_ << x,y;
			}
			void init(DataType x, DataType y, DataType z)
			{
				data_ << x,y,z;
			}
			/**
				\brief Returns the length of this point
			*/
			inline DataType length()
			{
				return static_cast<DataType>(data_.norm());
			}
			/**
				\brief Returns the maximum of two points p1 and p2
			*/
			static const Point<DataType,dims>& max(const Point<DataType,dims>& p1, const Point<DataType,dims>& p2)
			{
				if (p1 > p2) {
					return p1;
				} else {
					return p2;
				}
			}
			/**
				\brief Returns the maximum value among the three axes of this point
			*/
			DataType max() const
			{
				return data_.array().max();
			}

			/**
				\brief Returns the minimum of two points p1 and p2
			*/
			static const Point<DataType,dims>& min(const Point<DataType,dims>& p1, const Point<DataType,dims>& p2)
			{
				if (p1 < p2) {
					return p1;
				} else {
					return p2;
				}
			}
			/**
				\brief Returns the minimum value among the three axes of this point
			*/
			DataType min() const
			{
				return data_.array().min();
			}

			/**
				\brief Gets the normalized value of this point (unitary)
			*/
			static const Point<DataType,dims>& norm(const Point<DataType,dims>& p)
			{
				Point<DataType,dims> q = p;
				q.data_.normalize();
				return q;
			}
			/**
				\brief Gets the normalized value of this point (unitary)
			*/
			Point<DataType,dims>& norm() const
			{
				Point<DataType,dims> p = *this;
				p.data_.normalize();
				return p;
			}

			/**
				\brief Output stream operator
			*/
			friend std::ostream& operator<<(std::ostream& out, const Point<DataType,dims>& p)
			{
				//Eigen::IOFormat OctaveFmt(Eigen::StreamPrecision, 0, ", ", ";\n", "", "", "[", "]");
				Eigen::IOFormat OctaveFmt(Eigen::StreamPrecision, 0, ",", ",", "", "", "[", "]");
				return out << p.data_.format(OctaveFmt);
			}

			/**
				\brief Returns a point which components are rounded to upper integer
				using ceil of the given point
			*/
			static const Point<int,dims> ceil(const Point<DataType,dims>& p)
			{
				Point<int,dims> q;
				q.data_.array() = p.data_.array().ceil().template cast < int >();
				return q;
			}
			/**
				\brief Returns a point which components are rounded to upper integer
				using ceil of this point
			*/
			Point<int,dims> ceil() const
			{
				Point<int,dims> q;
				q.data_.array() = data_.array().ceil().template cast < int >();
				return q;
			}
			/**
				\brief Returns a point which components are the absolute value
				of the given point
			*/
			static const Point<DataType,dims> abs(const Point<DataType,dims>& p)
			{
				Point<DataType,dims> q;
				q.data_.array() = p.data_.array().abs();
				return q;
			}
			/**
				\brief Returns a point which components are the absolute value
				of this point
			*/
			Point<DataType,dims> abs() const
			{
				Point<DataType,dims> q;
				q.data_.array() = data_.array().abs();
				return q;
			}
			/**
				\brief Returns a point which components are fixed from INFINITE and NAN
				errors, using the given point
			*/
			static Point<DataType,dims> fix(const Point<DataType,dims>& p)
			{
				Point<DataType,dims> q;
				q.data_.array() = p.data_.array().isFinite().select(p.data_, 0);
				return q;
			}
			/**
				\brief Fixes this point from INFINITE and NAN errors
			*/
			void fix()
			{
				data_.array().isFinite().select(data_, 0);
			}
			/**
				\brief Returns a point which components are the inverse
				of the given point
			*/
			static Point<DataType,dims> inverse(const Point<DataType,dims>& p)
			{
				Point<DataType,dims> q;
				q.data_.array() = p.data_.array().inverse();
				return q;
			}
			/**
				\brief Returns a point which components are the inverse
				of this point
			*/
			Point<DataType,dims> inverse() const
			{
				Point<DataType,dims> q;
				q.data_.array() = data_.array().inverse();
				return q;
			}
			/**
				\brief Returns a point which components are the sign value
				(0 for positive, 1 for negative) of the given point
			*/
			static Point<DataType,dims> sign(const Point<DataType,dims>& p)
			{
				Point<DataType,dims> q;
				q.data_.array() = (p.data_.array() < 0).select(Vector_t::Constant(dims,1),Vector_t::Constant(dims,0));
				return q;
			}
			/**
				\brief Returns a point which components are the sign value
				(0 for positive, 1 for negative) of this point
			*/
			Point<DataType,dims> sign() const
			{
				Point<DataType,dims> q;
				q.data_.array() = (data_.array() < 0).select(Vector_t::Constant(dims,1),Vector_t::Constant(dims,0));
				return q;
			}
			/**
				\brief Returns a point which components are the sign value
				(1 for positive, -1 for negative, 0 for zero) of this point
			*/
			Point<DataType,dims> sign2() const
			{
				Point<DataType,dims> q;
				q.data_.array() = data_.array().sign();
				return q;
			}
			/**
				\brief Typecasting for point elements
			*/
			template <typename DataTypeOther, int dimensions = dims>
			Point<DataTypeOther,dimensions> cast() const
			{
				Point<DataTypeOther,dimensions> q;
				for (int i=0; i<q.data_.rows(); i++) {
					q.data_(i) = static_cast<DataTypeOther>(data_(i));
				}
				return q;
			}
			/**
				\brief Ascending sorting of point elements
			*/
			Point<DataType,dims> sort() const
			{
				Point<DataType,dims> q;
				q.data_.array() = data_.array();
				std::sort(q.data_.data(), q.data_.data()+q.data_.size());
				return q;
			}
	};

	template<typename DataType>
	using Point2D = Point<DataType,2>;

	template<typename DataType>
	using Point3D = Point<DataType,3>;

} //namespace

#endif /* POINT_H_ */
