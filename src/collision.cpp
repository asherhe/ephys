#include "ephys/collision.h"
#include "ephys/rigidbody.h"

#include <cmath>
#include <list>

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
  Vec2 c1Center = c1.body->local2World(c1.origin()),
       c2Center = c2.body->local2World(c2.origin());

  return (c2Center - c1Center).norm() < c1.radius + c2.radius;
}

bool IntersectionDetector::boxCircle(const BoxCollider &bc, const CircleCollider &cc)
{
  Vec2 center = bc.object2Collider(bc.body->world2Local(cc.body->local2World(cc.origin())));

  bool right = center.x > bc.halfSize.x;
  bool left = center.x < -bc.halfSize.x;
  bool top = center.y > bc.halfSize.y;
  bool bottom = center.y < -bc.halfSize.y;

  if (top)
  {
    // center top-left/right
    if (right)
      return (center - bc.halfSize).norm() < cc.radius;
    if (left)
      return (center - Vec2(-bc.halfSize.x, bc.halfSize.y)).norm() < cc.radius;
    // center above
    return center.y - bc.halfSize.y < cc.radius;
  }
  if (bottom)
  {
    // center bottom-left/right
    if (right)
      return (center - Vec2(bc.halfSize.x, -bc.halfSize.y)).norm() < cc.radius;
    if (left)
      return (center + bc.halfSize).norm() < cc.radius;
    // center below
    return center.y + bc.halfSize.y > -cc.radius;
  }

  // center to the right/left
  if (right)
    return center.x - bc.halfSize.x < cc.radius;
  if (left)
    return center.x + bc.halfSize.x > -cc.radius;

  // center inside of box
  return true;
}

struct Range
{
public:
  float min, max;
  Range() : min(0), max(0) {}
  Range(float min, float max) : min(min), max(max) {}
  Range(const Range &r) : min(r.min), max(r.max) {}

  inline float center() const { return (min + max) / 2; }

  // check for overlap
  inline bool operator&&(const Range &other) const { return min <= other.max && other.min <= max; }

  inline float penetration(const Range &other) const
  {
    if (other.center() < center())
      return other.max - min;
    else
      return max - other.min;
  }
};

// determines the ranges two boxes occupy when projected onto the normal
// returns an array containing the range for b1 and b2, respectively
Range *boxBoxSat(Vec2 normal, Vec2 b1[4], Vec2 b2[4],
                 size_t *b1iMin = nullptr, size_t *b1iMax = nullptr,
                 size_t *b2iMin = nullptr, size_t *b2iMax = nullptr)
{
  float b1Min = INFINITY, b1Max = -INFINITY;
  float b2Min = INFINITY, b2Max = -INFINITY;

  size_t i1Min, i1Max;
  size_t i2Min, i2Max;

  for (int i = 0; i < 4; ++i)
  {
    float proj = normal * b1[i];
    if (proj < b1Min)
    {
      b1Min = proj;
      i1Min = i;
    }
    if (proj > b1Max)
    {
      b1Max = proj;
      i1Max = i;
    }
  }

  for (int i = 0; i < 4; ++i)
  {
    float proj = normal * b2[i];
    if (proj < b2Min)
    {
      b2Min = proj;
      i2Min = i;
    }
    if (proj > b2Max)
    {
      b2Max = proj;
      i2Max = i;
    }
  }

  if (b1iMin)
    *b1iMin = i1Min;
  if (b1iMax)
    *b1iMax = i1Max;
  if (b2iMin)
    *b2iMin = i2Min;
  if (b2iMax)
    *b2iMax = i2Max;

  Range *ranges = new Range[2];
  ranges[0] = Range(b1Min, b1Max);
  ranges[1] = Range(b2Min, b2Max);

  return ranges;
}

bool IntersectionDetector::boxBox(const BoxCollider &b1, const BoxCollider &b2)
{
  // box vertex ids
  // 1--0
  // |  |
  // 3--2

  // list of vertices
  Vec2 bv[2][4];
  bv[0][0] = b1.halfSize;
  bv[0][1] = Vec2(-b1.halfSize.x, b1.halfSize.y);
  bv[0][2] = Vec2(b1.halfSize.x, -b1.halfSize.y);
  bv[0][3] = -b1.halfSize;
  bv[1][0] = b2.halfSize;
  bv[1][1] = Vec2(-b2.halfSize.x, b2.halfSize.y);
  bv[1][2] = Vec2(b2.halfSize.x, -b2.halfSize.y);
  bv[1][3] = -b2.halfSize;

  // convert b2 vertices to b1 collider space
  Mat3 b2tob1 = b2.getInvTransform() * b1.body->getInvTransform() * b2.body->getTransform() * b2.getTransform();
  for (int i = 0; i < 4; ++i)
    bv[1][i] = b2tob1 * bv[1][i];

  Range *satRanges;

  // separating axis: b1 y-axis (x normal)
  satRanges = boxBoxSat(Vec2(1, 0), bv[0], bv[1]);
  if (satRanges[0] && satRanges[1])
    return true;

  // separating axis: b1 x-axis (y normal)
  satRanges = boxBoxSat(Vec2(0, 1), bv[0], bv[1]);
  if (satRanges[0] && satRanges[1])
    return true;

  // separating axis: b2 y-axis (x normal)
  satRanges = boxBoxSat(bv[1][0] - bv[1][1], bv[0], bv[1]);
  if (satRanges[0] && satRanges[1])
    return true;

  // separating axis: b2 x-axis (y normal)
  satRanges = boxBoxSat(bv[1][0] - bv[1][2], bv[0], bv[1]);
  if (satRanges[0] && satRanges[1])
    return true;

  return false;
}

std::list<Contact> &CollisionDetector::circleCircle(const CircleCollider &c1, const CircleCollider &c2, const CollisionProperties &properties)
{
  auto contacts = new std::list<Contact>;

  Vec2 c1Center = c1.body->local2World(c1.origin()),
       c2Center = c2.body->local2World(c2.origin()),
       displacement = c2Center - c1Center;
  float distance = displacement.norm(),
        circleSpace = distance - (c1.radius + c2.radius);

  if (circleSpace > 0)
    return *contacts;

  Contact contact;
  contact.bodies[0] = c1.body;
  contact.bodies[1] = c2.body;
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

  Contact contact;
  contact.bodies[0] = bc.body;
  contact.bodies[1] = cc.body;
  contact.restitution = properties.restitution;

  Vec2 center = bc.object2Collider(bc.body->world2Local(cc.body->local2World(cc.origin())));

  bool hasContact = false, isCorner = false;

  bool left = center.x < -bc.halfSize.x;
  bool right = center.x > bc.halfSize.x;
  bool top = center.y > bc.halfSize.y;
  bool bottom = center.y < -bc.halfSize.y;

  Vec2 corner(bc.halfSize);

  if (top)
  {
    // center in top-left/right corner
    if ((right && (center - bc.halfSize).norm() < cc.radius) || (left && (center - corner.set(-bc.halfSize.x, bc.halfSize.y)).norm() < cc.radius))
      isCorner = true;
    else if (center.y - bc.halfSize.y < cc.radius) // center above box
    {
      contact.contactPoint.set(center.x, bc.halfSize.y);
      contact.normal.set(0, 1);
      contact.penetration = (center.y - bc.halfSize.y) - cc.radius;
      hasContact = true;
    }
  }
  else if (bottom)
  {
    // center in bottom-left/right corner
    if ((right && (center - corner.set(bc.halfSize.x, -bc.halfSize.y)).norm() < cc.radius) || (left && (center - (corner = -bc.halfSize)).norm() < cc.radius))
      isCorner = true;
    else if (center.y + bc.halfSize.y > -cc.radius) // center below box
    {
      contact.contactPoint.set(center.x, -bc.halfSize.y);
      contact.normal.set(0, -1);
      contact.penetration = cc.radius - (bc.halfSize.y + center.y);
      hasContact = true;
    }
  }
  else
  {
    // center to the right/left
    if (right && center.x - bc.halfSize.x < cc.radius)
    {
      contact.contactPoint.set(bc.halfSize.x, center.y);
      contact.normal.set(1, 0);
      contact.penetration = (center.x - bc.halfSize.x) - cc.radius;
      hasContact = true;
    }
    else if (left && center.x + bc.halfSize.x > -cc.radius)
    {
      contact.contactPoint.set(-bc.halfSize.x, center.y);
      contact.normal.set(-1, 0);
      contact.penetration = cc.radius - (center.x + bc.halfSize.x);
      hasContact = true;
    }
    else // center inside of box
    {
      contact.contactPoint = center;

      float toRight = bc.halfSize.x - center.x,
            toLeft = center.x + bc.halfSize.x,
            toTop = bc.halfSize.y - center.y,
            toBottom = center.y + bc.halfSize.x;

      float minDistanceX = fmin(toRight, toLeft),
            minDistanceY = fmin(toTop, toBottom);

      if (minDistanceX < minDistanceY)
      {
        if (minDistanceX == toRight)
          contact.normal.set(1, 0);
        else
          contact.normal.set(-1, 0);
        contact.penetration = minDistanceX;
      }
      else
      {
        if (minDistanceY == toTop)
          contact.normal.set(0, 1);
        else
          contact.normal.set(0, -1);
        contact.penetration = minDistanceY;
      }

      hasContact = true;
    }
  }

  if (isCorner)
  {
    contact.contactPoint = corner;
    Vec2 displacement = center - corner;
    float distance = displacement.norm();
    contact.normal = displacement / distance;
    contact.penetration = cc.radius - distance;
    hasContact = true;
  }

  if (hasContact)
  {
    // convert contact point and normal to world coords
    contact.contactPoint = bc.body->local2World(bc.collider2Object(contact.contactPoint));
    contact.normal = bc.body->rotLocal2World(bc.rotCollider2Object(contact.normal));

    contacts->push_back(contact);
  }

  return *contacts;
}

// clips v1 and v2 such that they are within a given distance along a normal
// returns false is both points are out of bounds
// otherwise, clipped output is written to v1Out and v2Out
bool clipPoints(Vec2 v1, Vec2 v2, Vec2 normal, float distance, Vec2 *v1Out, Vec2 *v2Out)
{
  float d1 = v1 * normal - distance,
        d2 = v2 * normal - distance;

  // both within bounds
  if (d1 >= 0)
    *v1Out = v1;
  if (d2 >= 0)
    *v2Out = v2;

  // both out of bounds
  if (d1 < 0 && d2 < 0)
    return false;

  // alternating signs - one of them is off the edge
  if (d1 * d2 < 0)
  {
    Vec2 clampVec = v2 - v1;
    clampVec *= d1 / (d1 - d2);
    clampVec += v1;

    if (d1 < 0)
      *v1Out = clampVec;
    if (d2 < 0)
      *v2Out = clampVec;
  }

  return true;
}

std::list<Contact> &CollisionDetector::boxBox(const BoxCollider &b1, const BoxCollider &b2, const CollisionProperties &properties)
{
  auto contacts = new std::list<Contact>;

  Vec2 bv[2][4];
  bv[0][0] = b1.halfSize;
  bv[0][1] = Vec2(-b1.halfSize.x, b1.halfSize.y);
  bv[0][2] = Vec2(b1.halfSize.x, -b1.halfSize.y);
  bv[0][3] = -b1.halfSize;
  bv[1][0] = b2.halfSize;
  bv[1][1] = Vec2(-b2.halfSize.x, b2.halfSize.y);
  bv[1][2] = Vec2(b2.halfSize.x, -b2.halfSize.y);
  bv[1][3] = -b2.halfSize;

  // convert b2 vertices to b1 collider space
  Mat3 b2tob1 = b2.getInvTransform() * b1.body->getInvTransform() * b2.body->getTransform() * b2.getTransform();
  for (int i = 0; i < 4; ++i)
    bv[1][i] = b2tob1 * bv[1][i];

  // identify axis with the least penetration
  enum Axis
  {
    NONE,
    B1_X,
    B1_Y,
    B2_X,
    B2_Y
  } contactAxis = NONE;
  Vec2 axisNormal, refV1, refV2;
  float penetration, minPenetration = INFINITY;
  Vec2 *penetrationVertex;

  Range *satRanges;
  size_t i1Min, i1Max, i2Min, i2Max;
  bool left;

  // separating axis: b1 x
  satRanges = boxBoxSat(Vec2(0, 1), bv[0], bv[1], &i1Min, &i1Max, &i2Min, &i2Max);
  if ((left = satRanges[0].center() < satRanges[1].center()))
    penetration = satRanges[0].max - satRanges[1].min;
  else
    penetration = satRanges[1].max - satRanges[0].min;

  if (penetration >= 0 && penetration < minPenetration)
  {
    minPenetration = penetration;
    contactAxis = B1_X;
    axisNormal.set(0, 1);
    penetrationVertex = bv[1] + (left ? i2Min : i2Max);
    if (left)
    {
      refV1 = bv[0][0];
      refV2 = bv[0][1];
    }
    else
    {
      axisNormal = -axisNormal;
      refV1 = bv[0][2];
      refV2 = bv[0][3];
    }
  }

  // separating axis: b1 y
  satRanges = boxBoxSat(Vec2(1, 0), bv[0], bv[1], &i1Min, &i1Max, &i2Min, &i2Max);
  if ((left = satRanges[0].center() < satRanges[1].center()))
    penetration = satRanges[0].max - satRanges[1].min;
  else
    penetration = satRanges[1].max - satRanges[0].min;

  if (penetration >= 0 && penetration < minPenetration)
  {
    minPenetration = penetration;
    contactAxis = B1_Y;
    axisNormal.set(1, 0);
    penetrationVertex = bv[1] + (left ? i2Min : i2Max);
    if (left)
    {
      refV1 = bv[0][0];
      refV2 = bv[0][2];
    }
    else
    {
      axisNormal = -axisNormal;
      refV1 = bv[0][1];
      refV2 = bv[0][3];
    }
  }

  // separating axis: b2 x
  Vec2 normal = (bv[1][0] - bv[1][2]).normalize();
  satRanges = boxBoxSat(normal, bv[0], bv[1], &i1Min, &i1Max, &i2Min, &i2Max);
  if ((left = satRanges[1].center() < satRanges[0].center()))
    penetration = satRanges[1].max - satRanges[0].min;
  else
    penetration = satRanges[0].max - satRanges[1].min;

  if (penetration >= 0 && penetration < minPenetration)
  {
    minPenetration = penetration;
    contactAxis = B2_X;
    axisNormal = normal;
    penetrationVertex = bv[0] + (left ? i1Min : i1Max);
    if (left)
    {
      refV1 = bv[1][0];
      refV2 = bv[1][1];
    }
    else
    {
      axisNormal = -axisNormal;
      refV1 = bv[1][2];
      refV2 = bv[1][3];
    }
  }

  // separating axis: b2 y
  normal.set(normal.y, -normal.x);
  satRanges = boxBoxSat(normal, bv[0], bv[1], &i1Min, &i1Max, &i2Min, &i2Max);
  if ((left = satRanges[1].center() < satRanges[0].center()))
    penetration = satRanges[1].max - satRanges[0].min;
  else
    penetration = satRanges[0].max - satRanges[1].min;

  if (penetration >= 0 && penetration < minPenetration)
  {
    minPenetration = penetration;
    contactAxis = B2_Y;
    axisNormal = normal;
    penetrationVertex = bv[0] + (left ? i1Min : i1Max);
    if (left)
    {
      refV1 = bv[1][0];
      refV2 = bv[1][2];
    }
    else
    {
      axisNormal = -axisNormal;
      refV1 = bv[1][1];
      refV2 = bv[1][3];
    }
  }

  // boxes do not intersect
  if (contactAxis == NONE)
    return *contacts;

  // determine incident face
  // which one is the incident face?
  // the one that is the most parallel to the reference face. in other words, the one with the smallest dot product with the normal
  Vec2 incV1, incV2;

  incV1 = *penetrationVertex;

  Vec2 *vNeighbors[2]; // neighbors of vertex
  if (contactAxis == B1_X || contactAxis == B1_Y)
    switch (penetrationVertex - bv[1])
    {
    case 0:
    case 3:
      vNeighbors[0] = &bv[1][1];
      vNeighbors[1] = &bv[1][2];
      break;
    case 1:
    case 2:
      vNeighbors[0] = &bv[1][0];
      vNeighbors[1] = &bv[1][3];
      break;
    }
  else
    switch (penetrationVertex - bv[1])
    {
    case 0:
    case 3:
      vNeighbors[0] = &bv[1][1];
      vNeighbors[1] = &bv[1][2];
      break;
    case 1:
    case 2:
      vNeighbors[0] = &bv[1][0];
      vNeighbors[1] = &bv[1][3];
      break;
    }

  if ((incV1 - *vNeighbors[0]) * axisNormal < (incV1 - *vNeighbors[1]) * axisNormal)
    incV2 = *vNeighbors[0];
  else
    incV2 = *vNeighbors[1];

  // clip points
  Vec2 refNorm = (refV2 - refV1);
  refNorm.normalize();

  if (!clipPoints(incV1, incV2, refNorm, refNorm * refV1, &incV1, &incV2) ||
      !clipPoints(incV1, incV2, -refNorm, -refNorm * refV1, &incV1, &incV2))
    return *contacts;

  // append only points that are inside the box
  // dot with normal; compare with face
  float faceDist = refV1 * axisNormal;

  if (incV1 * axisNormal < faceDist)
  {
    Contact contact;
    contact.bodies[0] = b1.body;
    contact.bodies[1] = b2.body;
    contact.contactPoint = b1.body->local2World(b1.collider2Object(incV1));
    contact.normal = b1.body->rotLocal2World(b1.rotCollider2Object(axisNormal));
    contact.restitution = properties.restitution;
    contacts->push_back(contact);
  }
  if (incV2 * axisNormal < faceDist)
  {
    Contact contact;
    contact.bodies[0] = b1.body;
    contact.bodies[1] = b2.body;
    contact.contactPoint = b1.body->local2World(b1.collider2Object(incV2));
    contact.normal = b1.body->rotLocal2World(b1.rotCollider2Object(axisNormal));
    contact.restitution = properties.restitution;
    contacts->push_back(contact);
  }

  return *contacts;
}
