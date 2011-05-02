/** @file Data.h
 * 
 * @author	Marek Kopicki (The University Of Birmingham)
 *
 * @version 1.0
 *
 */

#pragma once
#ifndef _GOLEM_TOOLS_DATA_H_
#define _GOLEM_TOOLS_DATA_H_

//------------------------------------------------------------------------------

#include <Golem/Tools/XMLParser.h>
#include <Golem/Math/Mat34.h>
#include <Golem/Math/Quat.h>
#include <Golem/Math/TriangleMesh.h>
#include <Golem/Math/Bounds.h>
#include <Golem/Tools/Context.h>

//------------------------------------------------------------------------------

namespace golem {

//------------------------------------------------------------------------------

/** Reads/writes Vec3 from/to a given context */
void XMLData(Vec3 &val, XMLContext* context, bool create = false);

/** Reads/writes Quat from/to a given context */
void XMLData(Quat &val, XMLContext* context, bool create = false);

/** Reads/writes Twist from/to a given context */
void XMLData(Twist &val, XMLContext* context, bool create = false);
void XMLData(Twist &val, Real& theta, XMLContext* context, bool create = false);

/** Reads/writes Mat33 from/to a given context */
void XMLData(Mat33 &val, XMLContext* context, bool create = false);

/** Reads/writes angle axis representation from/to a given context */
void XMLDataAngleAxis(Real &angle, Vec3 &axis, XMLContext* context, bool create = false);

/** Reads/writes Euler representation from/to a given context */
void XMLDataEuler(Real& roll, Real& pitch, Real& yaw, XMLContext* context, bool create = false);

/** Reads/writes Mat34 from/to a given context */
void XMLData(Mat34 &val, XMLContext* context, bool create = false);

//------------------------------------------------------------------------------

/** Reads/writes triangle from/to a given context */
void XMLData(Triangle &val, XMLContext* context, bool create = false);

//------------------------------------------------------------------------------
// Bounds support - binary streams.

/** Loads bounds description from the stream  */
template <> void Stream::read(Bounds::Desc::Ptr& desc) const;

template <> void Stream::read(BoundingPlane::Desc& desc) const;
template <> void Stream::read(BoundingSphere::Desc& desc) const;
template <> void Stream::read(BoundingCylinder::Desc& desc) const;
template <> void Stream::read(BoundingBox::Desc& desc) const;
template <> void Stream::read(BoundingConvexMesh::Desc& desc) const;

/** Loads bounds from the stream  */
template <> void Stream::read(Bounds::Ptr& bounds) const;

/** Stores bounds description to the stream  */
template <> void Stream::write(const Bounds::Desc::Ptr& desc);

template <> void Stream::write(const BoundingPlane::Desc& desc);
template <> void Stream::write(const BoundingSphere::Desc& desc);
template <> void Stream::write(const BoundingCylinder::Desc& desc);
template <> void Stream::write(const BoundingBox::Desc& desc);
template <> void Stream::write(const BoundingConvexMesh::Desc& desc);

/** Stores bounds to the stream  */
template <> void Stream::write(const Bounds::Ptr& bounds);

//------------------------------------------------------------------------------
// Bounds support - xml format.

class XMLContext;

/** Reads/writes bounds from/to a given context */
void XMLData(Bounds::Desc::Ptr &val, XMLContext* context, bool create = false);
void XMLData(Bounds::Desc &val, XMLContext* context, bool create = false);

void XMLData(BoundingPlane::Desc &val, XMLContext* context, bool create = false);
void XMLData(BoundingSphere::Desc &val, XMLContext* context, bool create = false);
void XMLData(BoundingCylinder::Desc &val, XMLContext* context, bool create = false);
void XMLData(BoundingBox::Desc &val, XMLContext* context, bool create = false);
void XMLData(BoundingConvexMesh::Desc &val, XMLContext* context, bool create = false);

//------------------------------------------------------------------------------

/** Message stream setup */
void XMLData(MessageStream::Desc::Ptr& desc, XMLContext* context, bool create = false);

/** Program context setup */
void XMLData(Context::Desc& desc, XMLContext* context, bool create = false);

//------------------------------------------------------------------------------

};	// namespace

#endif /*_GOLEM_TOOLS_DATA_H_*/
