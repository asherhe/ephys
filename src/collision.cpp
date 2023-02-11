#include "ephys/collision.h"
#include "ephys/collider.h"
#include "ephys/rigidbody.h"

#include <cmath>
#include <list>
#include <map>
#include <typeinfo>
#include <algorithm>

#include <iostream>

using namespace ephys;

BoundingCircle::BoundingCircle(const BoundingCircle &b1, const BoundingCircle &b2)
{
  Vec2 dCenter = b2.center - b1.center;
  float distance = dCenter.norm();

  if (b1.radius >= distance + b2.radius) // b2 is in b1
  {
    center = b1.center;
    radius = b1.radius;
  }
  else if (b2.radius >= distance + b1.radius) // b1 is in b2
  {
    center = b2.center;
    radius = b2.radius;
  }
  else // partially or not at all overlapping volumes
  {
    radius = (b1.radius + distance + b2.radius) / 2;
    center = b1.center + dCenter * (radius - b1.radius) / distance;
  }
}

bool BoundingCircle::overlaps(const BoundingVolume &bv) const
{
  const BoundingCircle &bc = dynamic_cast<const BoundingCircle &>(bv);

  return (center - bc.center).norm() < radius + bc.radius;
}

bool IntersectionDetector::circleCircle(const CircleCollider &c1, const CircleCollider &c2)
{
  Vec2 c1Center = c1.getBody()->local2World(c1.origin()),
       c2Center = c2.getBody()->local2World(c2.origin());

  return (c2Center - c1Center).norm() < c1.radius + c2.radius;
}

bool IntersectionDetector::boxCircle(const BoxCollider &bc, const CircleCollider &cc)
{
  // in bc space
  Vec2 center = bc.object2Collider(bc.getBody()->world2Local(cc.getBody()->local2World(cc.origin())));

  Vec2 closest = center;

  if (closest.x > bc.halfSize.x)
    closest.x = bc.halfSize.x;
  else if (closest.x < -bc.halfSize.x)
    closest.x = -bc.halfSize.x;

  if (closest.y > bc.halfSize.y)
    closest.y = bc.halfSize.y;
  else if (closest.y < -bc.halfSize.y)
    closest.y = -bc.halfSize.y;

  return (center - closest).norm() <= cc.radius;
}

bool IntersectionDetector::boxBox(const BoxCollider &b1, const BoxCollider &b2)
{
  Mat2 rot1 = (b1.getBody()->getTransform() * b1.getTransform()).getRotation(),
       rot2 = (b2.getBody()->getTransform() * b2.getTransform()).getRotation(),
       invRot1 = rot1.transpose(),
       invRot2 = rot2.transpose();

  Vec2 pos1 = b1.getBody()->local2World(b1.origin()),
       pos2 = b2.getBody()->local2World(b2.origin()),
       displacement = pos2 - pos1,
       displacement1 = invRot1 * displacement,
       displacement2 = invRot2 * displacement;

  Mat2 rotB2toB1 = invRot1 * rot2,
       absRotB2toB1 = rotB2toB1.abs(),
       absRotB2toB1T = absRotB2toB1.transpose();

  Vec2 sat1 = displacement1.abs() - b1.halfSize - absRotB2toB1 * b2.halfSize;
  if (sat1.x > 0 || sat1.y > 0) // no penetration
    return false;
  Vec2 sat2 = displacement2.abs() - b2.halfSize - absRotB2toB1T * b1.halfSize;
  if (sat2.x > 0 || sat2.y > 0) // no penetration
    return false;

  return true;
}

std::list<Contact> &CollisionDetector::circleCircle(const CircleCollider &c1, const CircleCollider &c2, const CollisionProperties &properties)
{
  auto contacts = new std::list<Contact>;

  Vec2 c1Center = c1.getBody()->local2World(c1.origin()),
       c2Center = c2.getBody()->local2World(c2.origin()),
       displacement = c2Center - c1Center;
  float distance = displacement.norm(),
        circleSpace = distance - (c1.radius + c2.radius);

  if (circleSpace > 0)
    return *contacts;

  Contact contact;
  contact.bodies[0] = c1.getBody();
  contact.bodies[1] = c2.getBody();
  contact.normal = displacement / distance;
  contact.contactPoint = c1Center + contact.normal * (c1.radius + circleSpace / 2);
  contact.penetration = -circleSpace;
  contact.restitution = properties.restitution;

  contacts->push_back(contact);
  return *contacts;
}

std::list<Contact> &CollisionDetector::boxCircle(const BoxCollider &bc, const CircleCollider &cc, const CollisionProperties &properties)
{
  auto contacts = new std::list<Contact>;

  Vec2 center = bc.object2Collider(bc.getBody()->world2Local(cc.getBody()->local2World(cc.origin())));

  Vec2 closest = center,
       normal;

  if (closest.x > bc.halfSize.x)
  {
    closest.x = bc.halfSize.x;
    ++normal.x;
  }
  else if (closest.x < -bc.halfSize.x)
  {
    closest.x = -bc.halfSize.x;
    --normal.x;
  }

  if (closest.y > bc.halfSize.y)
  {
    closest.y = bc.halfSize.y;
    ++normal.y;
  }
  else if (closest.y < -bc.halfSize.y)
  {
    closest.y = -bc.halfSize.y;
    --normal.y;
  }

  if (normal.x == 0 && normal.y == 0)
  {
    if (center.x == 0 && center.y == 0)
      normal.set(0, 1);
    else
      normal = center;
  }
  normal.normalize();

  Vec2 displacement = center - closest;
  float distance = displacement.norm();
  if (distance > cc.radius)
    return *contacts;

  Contact contact;
  Mat3 bcToWorld = bc.getBody()->getTransform() * bc.getTransform();
  contact.bodies[0] = bc.getBody();
  contact.bodies[1] = cc.getBody();
  contact.contactPoint = bcToWorld * closest;
  contact.normal = bcToWorld.getRotation() * normal;
  contact.penetration = distance - cc.radius;
  contact.restitution = properties.restitution;

  contacts->push_back(contact);

  return *contacts;
}

enum Edge
{
  NONE,
  E1,
  E2,
  E3,
  E4
};

struct ClipPoint
{
  Vec2 v;
  Edge inEdge1, inEdge2, outEdge1, outEdge2;
};

// determine which face is the incident face
void computeIncidentFace(ClipPoint incFace[2], const Vec2 &halfSize, const Vec2 &pos, const Mat2 &rot, const Vec2 &frontNormal)
{
  Mat2 rotT = rot.transpose();
  Vec2 n = -(rotT * frontNormal),
       absN = n.abs();

  if (absN.x > absN.y)
  {
    if (n.x > 0)
    {
      incFace[0].v.set(halfSize.x, -halfSize.y);
      incFace[0].inEdge2 = E3;
      incFace[0].outEdge2 = E4;

      incFace[1].v.set(halfSize.x, halfSize.y);
      incFace[1].inEdge2 = E4;
      incFace[1].outEdge2 = E1;
    }
    else
    {
      incFace[0].v.set(-halfSize.x, halfSize.y);
      incFace[0].inEdge2 = E1;
      incFace[0].outEdge2 = E2;

      incFace[1].v.set(-halfSize.x, -halfSize.y);
      incFace[1].inEdge2 = E2;
      incFace[1].outEdge2 = E3;
    }
  }
  else
  {
    if (n.y > 0)
    {
      incFace[0].v.set(halfSize.x, halfSize.y);
      incFace[0].inEdge2 = E4;
      incFace[0].outEdge2 = E1;

      incFace[1].v.set(-halfSize.x, halfSize.y);
      incFace[1].inEdge2 = E1;
      incFace[1].outEdge2 = E2;
    }
    else
    {
      incFace[0].v.set(-halfSize.x, -halfSize.y);
      incFace[0].inEdge2 = E2;
      incFace[0].outEdge2 = E3;

      incFace[1].v.set(halfSize.x, -halfSize.y);
      incFace[1].inEdge2 = E3;
      incFace[1].outEdge2 = E4;
    }
  }

  incFace[0].v = pos + rot * incFace[0].v;
  incFace[1].v = pos + rot * incFace[1].v;
}

size_t clipSegment(ClipPoint vout[2], ClipPoint vin[2], const Vec2 &normal, float offset, Edge clipEdge)
{
  // initially no output points
  size_t numOut = 0;

  // distance of endpoints along the normal
  float distance0 = normal * vin[0].v - offset,
        distance1 = normal * vin[1].v - offset;

  // points are behind the plane
  if (distance0 <= 0)
    vout[numOut++] = vin[0];
  if (distance1 <= 0)
    vout[numOut++] = vin[1];

  // points are on opposite sides of the plane
  if (distance0 * distance1 < 0)
  {
    float intersection = distance0 / (distance0 - distance1);
    vout[numOut].v = vin[0].v + intersection * (vin[1].v - vin[0].v);
    if (distance0 > 0)
    {
      vout[numOut].inEdge1 = clipEdge;
      vout[numOut].inEdge2 = NONE;
      vout[numOut].outEdge1 = vin[0].outEdge1;
      vout[numOut].outEdge2 = vin[0].outEdge1;
    }
    else
    {
      vout[numOut].inEdge1 = clipEdge;
      vout[numOut].inEdge2 = NONE;
      vout[numOut].outEdge1 = vin[1].outEdge1;
      vout[numOut].outEdge2 = vin[1].outEdge1;
    }
    ++numOut;
  }

  return numOut;
}

std::list<Contact> &CollisionDetector::boxBox(const BoxCollider &b1, const BoxCollider &b2, const CollisionProperties &properties)
{
  std::cout << "box box\n";

  // box vertex/edge ids
  //     e0
  //    1--0
  // e1 |  | e3
  //    3--2
  //     e2

  auto contacts = new std::list<Contact>;

  // rot - collider to world
  // invRot - world to collider
  Mat2 rot1 = (b1.getBody()->getTransform() * b1.getTransform()).getRotation(),
       rot2 = (b2.getBody()->getTransform() * b2.getTransform()).getRotation(),
       invRot1 = rot1.transpose(),
       invRot2 = rot2.transpose();

  Vec2 pos1 = b1.getBody()->local2World(b1.origin()),
       pos2 = b2.getBody()->local2World(b2.origin()),
       displacement = pos2 - pos1,
       displacement1 = invRot1 * displacement,
       displacement2 = invRot2 * displacement;

  Mat2 rotB2toB1 = invRot1 * rot2,
       absRotB2toB1 = rotB2toB1.abs(),
       absRotB2toB1T = absRotB2toB1.transpose();

  // SAT penetration for both boxes
  Vec2 sat1 = displacement1.abs() - b1.halfSize - absRotB2toB1 * b2.halfSize;
  if (sat1.x > 0 || sat1.y > 0) // no penetration
    return *contacts;
  Vec2 sat2 = displacement2.abs() - b2.halfSize - absRotB2toB1T * b1.halfSize;
  if (sat2.x > 0 || sat2.y > 0) // no penetration
    return *contacts;

  // determine axis of least penetration
  enum Axis
  {
    B1X,
    B1Y,
    B2X,
    B2Y
  } axis;
  // minimum penetration
  float separation;
  Vec2 normal;

  axis = B1X;
  separation = sat1.x;
  normal = displacement1.x > 0 ? rot1.column(0) : -rot1.column(0);

  if (sat1.y > separation)
  {
    axis = B1Y;
    separation = sat1.y;
    normal = displacement1.y > 0 ? rot1.column(1) : -rot1.column(1);
  }

  if (sat2.x > separation)
  {
    axis = B2X;
    separation = sat2.x;
    normal = displacement2.x > 0 ? rot2.column(0) : -rot2.column(0);
  }

  if (sat2.y > separation)
  {
    axis = B2Y;
    separation = sat2.y;
    normal = displacement2.y > 0 ? rot2.column(1) : -rot2.column(1);
  }

  Vec2 frontNormal, sideNormal;
  ClipPoint incFace[2];
  float front, side, posSide, negSide;
  Edge posEdge, negEdge;

  switch (axis)
  {
  case B1X:
    frontNormal = normal;
    sideNormal = rot1.column(1);
    front = pos1 * frontNormal + b1.halfSize.x;
    side = pos1 * sideNormal;
    posSide = b1.halfSize.y + side;
    negSide = b1.halfSize.y - side;
    posEdge = E3;
    negEdge = E1;
    computeIncidentFace(incFace, b2.halfSize, pos2, rot2, frontNormal);
    break;
  case B1Y:
    frontNormal = normal;
    sideNormal = rot1.column(0);
    front = pos1 * frontNormal + b1.halfSize.y;
    side = pos1 * sideNormal;
    posSide = b1.halfSize.x + side;
    negSide = b1.halfSize.x - side;
    posEdge = E2;
    negEdge = E4;
    computeIncidentFace(incFace, b2.halfSize, pos2, rot2, frontNormal);
    break;
  case B2X:
    frontNormal = -normal;
    sideNormal = rot2.column(1);
    front = pos2 * frontNormal + b2.halfSize.x;
    side = pos2 * sideNormal;
    posSide = b2.halfSize.y + side;
    negSide = b2.halfSize.y - side;
    posEdge = E3;
    negEdge = E1;
    computeIncidentFace(incFace, b1.halfSize, pos1, rot1, frontNormal);
    break;
  case B2Y:
    frontNormal = -normal;
    sideNormal = rot2.column(0);
    front = pos2 * frontNormal + b2.halfSize.y;
    side = pos2 * sideNormal;
    posSide = b2.halfSize.x + side;
    negSide = b2.halfSize.x - side;
    posEdge = E2;
    negEdge = E4;
    computeIncidentFace(incFace, b1.halfSize, pos1, rot1, frontNormal);
    break;
  }

  // TODO: https://github.com/erincatto/box2d-lite/blob/master/src/Collide.cpp#L289
  ClipPoint cp1[2], cp2[2];
  int numPoints;

  // clip box side 1
  numPoints = clipSegment(cp1, incFace, -sideNormal, negSide, negEdge);
  if (numPoints < 2)
    return *contacts;

  // clip box side 2
  numPoints = clipSegment(cp2, cp1, sideNormal, posSide, posEdge);
  if (numPoints < 2)
    return *contacts;

  std::cout << "  clip\n"
            << "    " << incFace[0].v << "-" << incFace[1].v << "\n"
            << "    " << cp1[0].v << "-" << cp1[1].v << "\n"
            << "    " << cp2[0].v << "-" << cp2[1].v << "\n";

  for (int i = 0; i < 2; ++i)
  {
    float separation = frontNormal * cp2[i].v - front;

    if (separation <= 0)
    {
      Contact contact;
      contact.bodies[0] = b1.getBody();
      contact.bodies[1] = b2.getBody();
      contact.contactPoint = cp2[i].v - 0.5 * separation * frontNormal;
      contact.normal = normal;
      contact.restitution = properties.restitution;
      contact.penetration = -separation;

      std::cout << "  vertex " << cp2[i].v << "\n"
                << "  front normal " << frontNormal << "\n"
                << "  front " << front << "\n"
                << "  penetration " << contact.penetration << "\n";

      contacts->push_back(contact);
    }
  }

  return *contacts;
}

std::list<Contact> &CollisionContactGenerator::generateContacts() const
{
  std::map<Rigidbody *, BoundingCircle> boundingVolumes;
  for (auto it = bodies->begin(); it != bodies->end(); ++it)
  {
    Vec2 center = (*it)->getPos();
    float radius = 1;

    const std::type_info &type = typeid(*((*it)->getCollider()));
    if (type == typeid(CircleCollider))
    {
      CircleCollider *cc = static_cast<CircleCollider *>((*it)->getCollider());
      center = cc->origin();
      radius = cc->radius;
    }
    else if (type == typeid(BoxCollider))
    {
      BoxCollider *bc = static_cast<BoxCollider *>((*it)->getCollider());
      center = bc->origin();
      radius = bc->halfSize.norm();
    }

    center = (*it)->local2World(center);

    std::cout << "  bv " << center << " " << radius << "\n";
    BoundingCircle bv(center, radius);

    boundingVolumes[*it] = bv;
  }

  auto it = boundingVolumes.begin();
  BVHNode<BoundingCircle> bvtree(it->second, it->first);
  for (++it; it != boundingVolumes.end(); ++it)
    bvtree.insert(it->second, it->first);

  auto potentialContacts = bvtree.getPotentialContacts();
  std::cout << potentialContacts.size() << " potential contacts\n";
  // remove duplicates
  potentialContacts.sort();
  potentialContacts.unique();

  auto contacts = new std::list<Contact>;

  for (auto it = potentialContacts.begin(); it != potentialContacts.end(); ++it)
  {
    short colliderFlags = 0;

    Collider *c0 = (it->bodies[0]->getCollider()),
             *c1 = (it->bodies[1]->getCollider());

    BoxCollider *bc0, *bc1;
    CircleCollider *cc0, *cc1;
    if (typeid(*c0) == typeid(BoxCollider))
    {
      colliderFlags |= 0b1;
      bc0 = static_cast<BoxCollider *>(c0);
    }
    else
      cc0 = static_cast<CircleCollider *>(c0);

    if (typeid(*c1) == typeid(BoxCollider))
    {
      colliderFlags |= 0b10;
      bc1 = static_cast<BoxCollider *>(c1);
    }
    else
      cc1 = static_cast<CircleCollider *>(c1);

    CollisionProperties properties;
    properties.restitution = it->bodies[0]->getRestitution();
    switch (colliderFlags)
    {
    case 0b00:
      contacts->splice(contacts->end(), CollisionDetector::circleCircle(*cc0, *cc1, properties));
      break;
    case 0b01:
      contacts->splice(contacts->end(), CollisionDetector::boxCircle(*bc0, *cc1, properties));
      break;
    case 0b10:
      contacts->splice(contacts->end(), CollisionDetector::boxCircle(*bc1, *cc0, properties));
      break;
    case 0b11:
      contacts->splice(contacts->end(), CollisionDetector::boxBox(*bc0, *bc1, properties));
      break;
    }
  }

  delete &potentialContacts;

  std::cout << contacts->size() << " contacts\n";

  return *contacts;
}
