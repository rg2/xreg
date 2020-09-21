/*
 * MIT License
 *
 * Copyright (c) 2020 Robert Grupp
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

/**
 * @file
 * @brief KD-Tree data structure along with utility functions and structs.
 *
 * Contains (along with the corresponding -inl.h file) the implemenation of
 * a generic KD-Tree for points and shapes as well as some basic types for
 * shapes and distance computations, traits for distinguishing points from
 * shapes, and other basic utility functions.
 **/

#ifndef XREGKDTREE_H_
#define XREGKDTREE_H_

#include <tuple>
#include <iterator>
#include <type_traits>

#include <boost/variant.hpp>

#include "xregCommon.h"
#include "xregTBBUtils.h"

namespace xreg
{

// begin forward declarations
struct TriMesh;
// end forward declarations

/**
 * @brief Triangle for use in a KD-Tree.
 *
 * Represents a triangle shape compatible for use in a KD-Tree. This stores
 * a pre-computed centroid of the triangle as well as pointers to each vertex.
 * The appropriate typedefs and methods are provided for compatibility as the
 * primary template argument to a KdTreeNode type.
 * @see KdTreeNode
 **/
struct KDTreeTri
{
  using Pt = Pt3;
  
  /**
   * @brief Triangle centroid
   *
   * The centroid of this triangle. This should be pre-computed by the user.
   **/
  Pt3 centroid; // pre-computed for speed

  /**
   * @brief Vertex pointers.
   *
   * Pointers to each of the vertices representing this triangle. Pointers are
   * used so that a surface may be deformed after the construction of the
   * KD-Tree (the KdTreeNode::update_bounds() method must be called to ensure
   * accurate lookups if vertices are modified).
   **/
  const Pt3* verts[3];

  /**
   * @brief Dimensionality.
   *
   * Returns the dimension of the point type; for most cases this will be 3.
   **/
  size_type size() const;

  /**
   * @brief Accessor for an element of the centroid.
   *
   * Retrieves an element of the triangle centroid.
   * @param The element index to retrieve
   * @return The appropriate element of the triangle centroid
   **/
  const CoordScalar& operator[](const size_type& dim) const;

  /**
   * @brief Find the closest point on this triangle.
   *
   * Finds the closest point between this triangle and a query point.
   * @param x The query point
   * @param tri_pt The closest point found on the this triangle
   * @param dist_obj Function object for computing distances between points
   * @param dist Stores the distance between the query point and the closest
   *             point on this triangle; null may be passed
   * @see xreg::FindClosestPtInTri
   **/
  std::tuple<Pt3,CoordScalar> closest_point(const Pt3& x) const;

  /**
   * @brief Calculate bounds for one dimension.
   *
   * Compute the bounding interval about this triangle along a specific
   * dimension.
   * @param min Non-null pointer to store the minimum value of the bounding interval
   * @param max Non-null pointer to store the maximum value of the bounding interval
   * @param dim The dimension used to compute the bounding interval (0-based)
   **/
  std::tuple<CoordScalar,CoordScalar> bounds(const size_type& dim) const;
};

/**
 * @brief Populates a list of triangles suitable for use by a KdTreeNode.
 *
 * Given a list of vertices and a list of triangles defined by vertex indices,
 * create a list of value type compatible with KdTreeTriangle.
 * @param verts List of vertices; the list implementation and list value type
 *              (point type) must overload the [] operator. verts[i][j]
 *              represents the jth component of the ith point.
 * @param tris List of triangles, where each triangle vertex is represented by
 *        its index in \p verts; The list implementation and value type must
 *        overload the [] operator. tris[i][j] represents the index into verts
 *        for jth vertex of the ith triangle; j must be in {0,1,2}. The list
 *        must provide a resize() method, which is used by this routine.
 * @param kd_tris Output list of data structures compatible with KdTreeTriangle.
 *
 **/
std::vector<KDTreeTri> CreateTrisForKDTree(const TriMesh& mesh);

/**
 * @brief Shape type-trait.
 *
 * Default type trait for determining if a class represents a shape. This
 * defaults to NOT a shape.
 **/
template <class T>
struct KDTreeIsShape
{
  /**
   * Boolean indicator for shape trait; 0 implies T is not a shape, and 1
   * implies T is a shape.
   **/
  constexpr static bool value = false;
};

/**
 * @brief Shape type-trait.
 *
 * Type trait to indicate that a KdTreeTriangle is a shape.
 **/
template<>
struct KDTreeIsShape<KDTreeTri>
{
  /**
   * Boolean indicator that KdTreeTriangle represents a shape.
   **/
  constexpr static bool value = true;
};

/**
 * @brief Retrieves the point type associated with a shape or point type.
 *
 * Type trait for obtaining the type of point when given either a shape or
 * an actual point. When given a point, the same point type is used.
 */
template <class T>
struct KDTreePointType
{
  using type = T;
};

template <>
struct KDTreePointType<KDTreeTri>
{
  using type = KDTreeTri::Pt;
};

/**
 * @brief A KD-Tree node; when created by the caller, it represents the root.
 *
 * KD-Tree implementation that may represent a collection of points or shapes
 * (e.g. from a Mesh). It is templated by the data type stored in each leaf
 * node; e.g. a point or shape, and an optional distance function object type.
 *
 * For example, KdTreeNode<Eigen::<Matrix<double,3,1> > defines a KD-Tree for
 * point data in \f$R^3\f$. KdTreeNode<KdTreeTriangle<Eigen::<Matrix<double,3,1> > >
 * defines a KD-Tree for triangular surface data in \f$R^3\f$.
 *
 * Some guidelines for creating shape/point types to use in this KD Tree:
 * -# Points
 *    - Must have the following typedefs
 *       - size_type
 *       - value_type (the type of the elements/components)
 *    - Must have a size() method indicating dimensionality
 *    - Must overload [] for component access
 * -# Shapes
 *    - Must have the following typedefs
 *       - size_type
 *       - value_type (the type of the shape vertices)
 *    - Must have a size() method indicating dimensionality
 *    - Must overload [] for component access to the point (usually the centroid)
 *      associated with this shape
 *    - Must implment a closest_point() method for calculating the closest point
 *      on this shape to a query point and the associated distance (distance
 *      can be weighted, as in the case with oriented points)
 *    - Must implement a bounds() method for computing a bounding box about the
 *      shape in a specified dimension (used when the KD tree is perturbed)
 *    - Must have a specialization of IsShape, with value = 1
 **/
template <class T>
class KDTreeNode
{
public:
  class UninitializedException { };
  
  using value_type = T;

  /**
   * @brief Point type
   *
   * The point type that is either stored in this tree or used to represent
   * point is space about shapes stored in this tree.
   */
  using Pt = typename KDTreePointType<value_type>::type;

  using PtList       = std::vector<Pt>;
  using DistList     = std::vector<CoordScalar>;
  using ShapePtrList = std::vector<const value_type*>;

  /**
   * @brief Const point type.
   * @see point_type
   **/
  //typedef const point_type const_point_type;

  /**
   * @brief The type represented by the elements of a point/vertex.
   *
   * Equivalently a scalar type. This is typically double or float.
   **/
  using Scalar = typename Pt::Scalar;

  /**
   * @brief Default constructor; creates an empty tree with invalid root node.
   **/
  KDTreeNode() = default;

  ~KDTreeNode() = default;

  /**
   * @brief Constructor. Constructs a KD-Tree from the input points/shapes.
   *
   * This calls init(pts_begin_it, pts_end_it, dim)
   * @param pts_begin_it Iterator to the start of the point collection
   * @param pts_end_it Iterator to the end of the point collection
   * @param dim The current dimension to sort about in this call, defaults to 0
   * @see init
   **/
  template<class Itr>
  KDTreeNode(Itr pts_begin_it, Itr pts_end_it, size_type dim = 0)
    : KDTreeNode()
  {
    init(pts_begin_it, pts_end_it, dim);
  }

  // No copying for now...
  KDTreeNode(const KDTreeNode&) = delete;
  KDTreeNode& operator=(const KDTreeNode&) = delete;

  /**
   * @brief Initializes a KD-Tree from the input points/shapes.
   *
   * This is a recursive creation process until leaf nodes are encountered. The
   * points are sorted about the current dimension and then split about the
   * median.
   * When a multithreading library is available, a parallel sorting algorithm
   * is used.
   * @param pts_begin_it Iterator to the start of the point collection
   * @param pts_end_it Iterator to the end of the point collection
   * @param dim The current dimension to sort about in this call, defaults to 0
   **/
  template<class Itr>
  void init(Itr pts_begin_it, Itr pts_end_it, size_type dim = 0)
  {
    static_assert(std::is_same<value_type, typename std::iterator_traits<Itr>::value_type>::value,
                  "Tree shape type differs from input shape type.");

    // resets all state, frees the memory used by all children
    clear();

    const size_type num_pts = std::distance(pts_begin_it, pts_end_it);

    if (num_pts > 1)
    {
      tbb::parallel_sort(pts_begin_it, pts_end_it,
                         [dim] (const value_type& x, const value_type& y)
                         {
                           return x[dim] < y[dim];
                         });

      // This chooses the median, could add some other options.
      const size_type div_ndx = num_pts / 2;

      NonLeafNodeData cur_data;

      cur_data.cur_dim = dim;
      cur_data.div_val = (*(pts_begin_it + div_ndx))[dim];

      // This will call an actual computation for shape data and set to zero for point data.
      // bounds are not used for point data
      
      std::tie(cur_data.left_min, cur_data.left_max) = FindBoundsHelper(pts_begin_it, pts_begin_it + div_ndx, dim);
      
      std::tie(cur_data.right_min, cur_data.right_max) = FindBoundsHelper(pts_begin_it + div_ndx, pts_end_it, dim);

      const size_type next_dim = (dim + 1) % (*pts_begin_it).size();

      cur_data.left.reset(new KDTreeNode(pts_begin_it, pts_begin_it + div_ndx, next_dim));
      cur_data.right.reset(new KDTreeNode(pts_begin_it + div_ndx, pts_end_it, next_dim));
      
      data_ = std::move(cur_data);
    }
    else if (num_pts != 0)
    {
      data_ = *pts_begin_it;
    }
    else
    {
      data_ = InvalidNodeData();
    }
  }

  /**
   * @brief Clears the tree of any contents.
   **/
  void clear()
  {
    data_ = InvalidNodeData();
  }

  /**
   * @brief Find the closest point represented by this tree to a given query point.
   *
   * For a KD-Tree storing only points, the closest point will be one of those
   * points. For a KD-Tree storing shapes, the closest point will be a point
   * on the surface of one of the stored shapes. This executes in a single
   * thread.
   * @param x The query point
   * @param dist (optional) Will store the distance from the query point to the
   *        closest point represented by the tree
   * @param shp (optional) Will store a pointer to the shape containing the
   *        closest point represented by the tree. For a non-shape tree
   *        this will store a pointer to a point equivalent to the return value.
   * @return The closest point to \p x represented by the tree
   **/
  std::tuple<Pt,CoordScalar> find_closest_point(const Pt& x, const value_type** shp = nullptr) const
  {
    find_closest_point_visitor visitor;
    visitor.query_pt = &x;
    visitor.shp      = shp;

    return boost::apply_visitor(visitor, data_);
  }

  /**
   * @brief Find the closest points, represented by this tree, to a list of
   * input query points.
   *
   * When a multithreading library is available the searches are
   * executed in parallel.
   * @param in_pts The list of input query points
   * @param out_pts The list of output closest points; may be in_pts
   **/
  void find_closest_points(const PtList& in_pts, PtList* out_pts) const
  {
    find_closest_points(in_pts, out_pts, nullptr, nullptr);
  }

  /**
   * @brief Find the closest points, represented by this tree, to a list of
   * input query points.
   *
   * Additionally, the distances between each query point and the
   * the closest point are passed back to the caller.
   * When a multithreading library is available the searches are
   * executed in parallel.
   * @param in_pts The list of input query points
   * @param out_pts The list of output closest points; may be in_pts
   * @param dists The list of distances between the each element of in_pts and
   *              the corresponding element in out_pts
   **/
  void find_closest_points(const PtList& in_pts, PtList* out_pts, DistList* dists) const
  {
    find_closest_points(in_pts, out_pts, dists, nullptr);
  }

  /**
   * @brief Find the closest points, represented by this tree, to a list of
   * input query points.
   *
   * The distances between each query point and the
   * the closest point are passed back to the caller. The shape data structures
   * are also passed back to the user in this call (making this call redundant
   * for trees of only point data).
   * When a multithreading library is available the searches are
   * executed in parallel.
   * @param in_pts The list of input query points
   * @param out_pts The list of output closest points; may be in_pts
   * @param dists The list of distances between the each element of in_pts and
   *              the corresponding element in out_pts
   * @param shps The dist of shapes associate with each closest point found
   **/
  void find_closest_points(const PtList& in_pts, PtList* out_pts,
                           DistList* dists, ShapePtrList* shps) const
  {
    const size_type num_pts = in_pts.size();

    xregASSERT(num_pts == out_pts->size());
    xregASSERT(!dists || (num_pts == dists->size()));
    xregASSERT(!shps  || (num_pts == shps->size()));

    auto find_closest_pts_helper = [this,in_pts, out_pts, dists, shps] (const RangeType& r)
    {
      CoordScalar tmp_dist = 0;

      for (size_type i = r.begin(); i != r.end(); ++i)
      {
        std::tie(out_pts->operator[](i), tmp_dist) = this->find_closest_point(in_pts[i], shps ? &shps->operator[](i) : nullptr);
        
        if (dists)
        {
          dists->operator[](i) = tmp_dist;
        }
      }
    };

    ParallelFor(find_closest_pts_helper, RangeType(0, num_pts));
  }

  /**
   * @brief Updates the internal bounding intervals.
   *
   * Updates the internal bounding intervals at this node and all of its
   * children by re-examining the point data (either a point for trees storing
   * only points, or vertices for trees storing shapes) represented at each node.
   * For example, this is used by the Active Shape Search Patient-to-Atlas
   * registration after new mode weights have been estimated and used to update
   * the current instance estimate.
   **/
  void update_bounds()
  {
    xregASSERT(KDTreeIsShape<value_type>::value);  // this should only be called when shapes are used
    
    update_bounds(nullptr, nullptr);
  }

  /**
   * @brief Returns the depth of the tree.
   *
   * This is re-computed on each call (the result is not stored).
   * @return The depth of the tree.
   **/
  size_type depth() const
  {
    return boost::apply_visitor(depth_visitor(), data_);
  }

  /// \brief Finds all points within a radius to the query point.
  void find_pts_in_radius(const Pt& x, const CoordScalar radius, PtList* pts, DistList* dists) const
  {
    xregASSERT(!KDTreeIsShape<value_type>::value);  // this should only be called when points are used

    pts->clear();
    dists->clear();

    find_pts_in_radius_helper(x, radius, pts, dists);
  }

  /// \brief Finds all points within a radius each query point in a collection.
  ///
  /// This is threaded with TBB when available.
  void find_pts_in_radius_for_pts(const PtList& query_pts, const CoordScalar radius,
                                  std::vector<PtList>* closest_pts,
                                  std::vector<DistList>* dists) const
  {
    const size_type num_query_pts = query_pts.size();
    
    xregASSERT(num_query_pts == closest_pts->size());
    xregASSERT(num_query_pts == dists->size());
    
    auto find_pts_in_radius_for_pts_helper = [this,query_pts,radius,closest_pts,dists] (const RangeType& r)
    {
      for (size_type i = r.begin(); i != r.end(); ++i)
      {
        this->find_pts_in_radius(query_pts[i], radius, &closest_pts->operator[](i), &dists->operator[](i));
      }
    };

    ParallelFor(find_pts_in_radius_for_pts_helper, RangeType(0, num_query_pts));
  }

private:
  void update_bounds(Pt* mins, Pt* maxs)
  {
    update_bounds_visitor visitor;
    visitor.mins = mins;
    visitor.maxs = maxs;

    boost::apply_visitor(visitor, data_);
  }

  void find_pts_in_radius_helper(const Pt& x, const CoordScalar radius, PtList* pts, DistList* dists) const
  {
    find_pts_in_radius_helper_visitor visitor;
    visitor.query_pt = &x;
    visitor.radius   = radius;
    visitor.pts      = pts;
    visitor.dists    = dists;
    
    boost::apply_visitor(visitor, data_);
  }

  struct NonLeafNodeData
  {
    // Information regarding the splitting plane at this node
    size_type cur_dim;
    Scalar div_val;
    
    // Pointers to child nodes
    std::unique_ptr<KDTreeNode> left;
    std::unique_ptr<KDTreeNode> right;
    
    // Bounding intervals about the child nodes for this node's current dimension
    Scalar left_min;
    Scalar left_max;
    Scalar right_min;
    Scalar right_max;
  };

  using LeafNodeData = value_type;
  
  struct InvalidNodeData { };

  // Using InvalidNodeData first defaults the node to be invalid upon default construction
  using NodeData = boost::variant<InvalidNodeData,NonLeafNodeData,LeafNodeData>;

  NodeData data_;
  
  // FindBoundsHelper for shape data
  template <class Itr>
  static
  typename std::enable_if<
    KDTreeIsShape<typename std::iterator_traits<Itr>::value_type>::value,
    std::tuple<CoordScalar,CoordScalar>>::type
  FindBoundsHelper(Itr begin_it, Itr end_it, const size_type dim)
  {
    CoordScalar cur_min;
    CoordScalar cur_max;
    CoordScalar tmp_min;
    CoordScalar tmp_max;

    std::tie(cur_min, cur_max) = begin_it->bounds(dim);

    for (Itr it = begin_it + 1; it != end_it; ++it)
    {
      std::tie(tmp_min, tmp_max) = it->bounds(dim);

      if (tmp_min < cur_min)
      {
        cur_min = tmp_min;
      }

      if (tmp_max > cur_max)
      {
        cur_max = tmp_max;
      }
    }

    return std::make_tuple(cur_min, cur_max);
  }

  // FindBoundsHelper for point data
  template <class Itr>
  static
  typename std::enable_if<
    !KDTreeIsShape<typename std::iterator_traits<Itr>::value_type>::value,
    std::tuple<CoordScalar,CoordScalar>>::type
  FindBoundsHelper(Itr begin_it, Itr end_it, const size_type dim)
  {
    return std::make_tuple(CoordScalar(0), CoordScalar(0));
  }

  // FindClosestPtHelper for shape data
  template <class S>
  static
  typename std::enable_if<KDTreeIsShape<S>::value,
                          std::tuple<Pt,CoordScalar>>::type
  FindClosestPtHelper(const Pt& query_pt, const S& leaf_data_shape)
  {
    return leaf_data_shape.closest_point(query_pt);
  }
  
  // FindClosestPtHelper for point data
  template <class S>
  static
  typename std::enable_if<!KDTreeIsShape<S>::value,
                          std::tuple<Pt,CoordScalar>>::type
  FindClosestPtHelper(const Pt& query_pt, const S& leaf_data_pt)
  {
    return std::make_tuple(leaf_data_pt, (query_pt - leaf_data_pt).norm());
  }

  struct find_closest_point_visitor : public boost::static_visitor<std::tuple<Pt,CoordScalar>>
  {
    const Pt* query_pt;

    const value_type** shp;

    Pt closest_pt;
    
    CoordScalar dist;

    std::tuple<Pt,CoordScalar> operator()(const InvalidNodeData&)
    {
      throw UninitializedException();
    }
    
    std::tuple<Pt,CoordScalar> operator()(const NonLeafNodeData& non_leaf_data)
    {
      const auto& x = *query_pt;
      
      const KDTreeNode*  first_child  = nullptr;
      const KDTreeNode*  second_child = nullptr;
      const CoordScalar* second_min   = nullptr;
      const CoordScalar* second_max   = nullptr;

      // A non leaf node will always have two valid children.

      if (x[non_leaf_data.cur_dim] < non_leaf_data.div_val)
      {
        first_child  = non_leaf_data.left.get();
        second_child = non_leaf_data.right.get();

        if (KDTreeIsShape<value_type>::value)
        {
          second_min = &non_leaf_data.right_min;
          second_max = &non_leaf_data.right_max;
        }
      }
      else
      {
        first_child  = non_leaf_data.right.get();
        second_child = non_leaf_data.left.get();

        if (KDTreeIsShape<value_type>::value)
        {
          second_min = &non_leaf_data.left_min;
          second_max = &non_leaf_data.left_max;
        }
      }

      Pt closest_pt;
      CoordScalar dist;

      std::tie(closest_pt,dist) = first_child->find_closest_point(x, shp);

      bool need_to_check = false;

      if (KDTreeIsShape<value_type>::value)
      {
        need_to_check = ((*second_min - dist) < x[non_leaf_data.cur_dim]) &&
                        (x[non_leaf_data.cur_dim] < (*second_max + dist));
      }
      else
      {
        need_to_check = std::abs(x[non_leaf_data.cur_dim] - non_leaf_data.div_val) <= dist;
      }

      if (need_to_check)
      {
        const value_type* sec_shp = nullptr;
        
        Pt sec_closest_pt;
        CoordScalar sec_dist;
        
        std::tie(sec_closest_pt,sec_dist) = second_child->find_closest_point(x, shp ? &sec_shp : nullptr);

        if (sec_dist < dist)
        {
          closest_pt = sec_closest_pt;
          dist       = sec_dist;

          if (shp)
          {
            *shp = sec_shp;
          }
        }
      }
      
      return std::make_tuple(closest_pt, dist);
    }
    
    std::tuple<Pt,CoordScalar> operator()(const LeafNodeData& leaf_data)
    {
      if (shp)
      {
        *shp = &leaf_data;
      }
      
      // This will do a copy when we're using point data in the KD Tree,
      // and do a more complicated computation when we're working with shape data.

      return FindClosestPtHelper(*query_pt, leaf_data);
    }
  };
 
  // This only works for fixed sized points
  struct update_bounds_visitor : public boost::static_visitor<void>
  {
    Pt* mins;
    Pt* maxs;
    
    void operator()(const InvalidNodeData&)
    {
      throw UninitializedException();
    }

    void operator()(NonLeafNodeData& non_leaf_data)
    {
      xregASSERT(!mins || !maxs || (mins->size() == maxs->size()));

      Pt left_mins;
      Pt left_maxs;
      Pt right_mins;
      Pt right_maxs;

      non_leaf_data.left->update_bounds(&left_mins, &left_maxs);
      non_leaf_data.right->update_bounds(&right_mins, &right_maxs);

      non_leaf_data.left_min  = left_mins[non_leaf_data.cur_dim];
      non_leaf_data.left_max  = left_maxs[non_leaf_data.cur_dim];
      non_leaf_data.right_min = right_mins[non_leaf_data.cur_dim];
      non_leaf_data.right_max = right_maxs[non_leaf_data.cur_dim];

      if (mins)
      {
        for (size_type i = 0; i < mins->size(); ++i)
        {
          mins->operator[](i) = std::min(left_mins[i], right_mins[i]);
        }
      }

      if (maxs)
      {
        for (size_type i = 0; i < maxs->size(); ++i)
        {
          maxs->operator[](i) = std::max(left_maxs[i], right_maxs[i]);
        }
      }
    }

    void operator()(const LeafNodeData& leaf_data)
    {
      for (size_type i = 0; i < leaf_data.size(); ++i)
      {
        std::tie(mins->operator[](i), maxs->operator[](i)) = FindBoundsHelper(&leaf_data, &leaf_data + 1, i);
      }
    }
  };

  struct depth_visitor : public boost::static_visitor<size_type>
  {
    size_type operator()(const InvalidNodeData&) const
    {
      return 0;
    }
    
    size_type operator()(const NonLeafNodeData& non_leaf_data) const
    {
      // for now this works; I am not allowing arbitrary node insertions, therefore
      // the tree is balanced and I only have to check one side
      return non_leaf_data.left->depth() + 1;
    }
    
    size_type operator()(const LeafNodeData& leaf_data) const
    {
      return 1;
    }
  };

  struct find_pts_in_radius_helper_visitor : public boost::static_visitor<void>
  {
    const Pt* query_pt;
    CoordScalar radius;
    PtList* pts;
    DistList* dists;

    void operator()(const InvalidNodeData&) const
    {
      throw UninitializedException();
    }
    
    void operator()(const NonLeafNodeData& non_leaf_data)
    {
      // Determine if we need to check the children, if the current point is far enough away
      // in one dimension, then we can exclude it.

      const auto& x = *query_pt;

      const bool need_to_check_both = std::abs(x[non_leaf_data.cur_dim] - non_leaf_data.div_val) <= radius;

      if (need_to_check_both)
      {
        non_leaf_data.left->find_pts_in_radius_helper(x, radius, pts, dists);
        non_leaf_data.right->find_pts_in_radius_helper(x, radius, pts, dists);
      }
      else if (x[non_leaf_data.cur_dim] <= non_leaf_data.div_val)
      {
        non_leaf_data.left->find_pts_in_radius_helper(x, radius, pts, dists);
      }
      else
      {
        non_leaf_data.right->find_pts_in_radius_helper(x, radius, pts, dists);
      }
    }
    
    void operator()(const LeafNodeData& leaf_data)
    {
      Pt closest_pt;
      CoordScalar dist;
      
      std::tie(closest_pt,dist) = FindClosestPtHelper(*query_pt, leaf_data);

      if (dist <= radius)
      {
        pts->push_back(closest_pt);
        dists->push_back(dist);
      }
    }
  };

};

}  // xreg

#endif
