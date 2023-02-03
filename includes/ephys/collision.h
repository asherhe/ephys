#ifndef EPHYS_COLLISION_H
#define EPHYS_COLLISION_H

#include "ephys/math.h"
#include "ephys/rigidbody.h"
#include "ephys/contacts.h"

#include <list>
#include <type_traits>

#include <cmath>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace ephys
{
  class BoundingVolume
  {
  public:
    virtual bool overlaps(const BoundingVolume &bv) const = 0;
    virtual float size() const = 0;
    // determine the amount for which the volume grows by if combined with another volume
    virtual float growth(const BoundingVolume &bv) const = 0;
  };

  class BoundingCircle : public BoundingVolume
  {
  public:
    Vec2 center;
    float radius;

    BoundingCircle(const Vec2 &center, float radius) : center(center), radius(radius) {}
    BoundingCircle(const BoundingCircle &b1, const BoundingCircle &b2);

    bool overlaps(const BoundingVolume &bv) const;

    inline float size() const
    {
      return M_PI * radius * radius;
    }

    inline float growth(const BoundingVolume &bv) const
    {
      const BoundingCircle &bc = dynamic_cast<const BoundingCircle &>(bv);
      float newRadius = ((center - bc.center).norm() + radius + bc.radius) / 2;
      return M_PI * (newRadius * newRadius - radius * radius);
    }
  };

  // a pair of rigidbodies athat are potentially in contact
  struct PotentialContact
  {
    Rigidbody *bodies[2];

    PotentialContact(Rigidbody &b1, Rigidbody &b2) : bodies{&b1, &b2} {}
  };

  template <typename BVClass>
  class BVHNode
  {
    static_assert(std::is_base_of<BoundingVolume, BVClass>::value, "bounding volume class must be derived from ephys::BoundingVolume");

  protected:
    std::list<PotentialContact *> &potentialContacts(BVHNode<BVClass> &n1, BVHNode<BVClass> &n2);
    void recalculateVolume();

  public:
    Rigidbody *body;
    BVClass volume;
    BVHNode *parent;
    BVHNode *children[2];

    BVHNode(const BVClass &volume, Rigidbody *body = nullptr) : volume(volume), body(body), parent(nullptr), children{NULL, NULL} {}
    ~BVHNode();

    inline bool isLeaf() const { return body != nullptr; }

    inline bool overlaps(const BVHNode &bvh) const { return volume.overlaps(bvh.volume); }

    void insert(Rigidbody &body, BVClass &volume);

    std::list<PotentialContact *> &getPotentialContacts() const;
  };

  template <typename BVClass>
  void BVHNode<BVClass>::recalculateVolume()
  {
    if (!isLeaf())
    {
      volume = BoundingVolume(children[0]->volume, children[1]->volume);

      if (parent)
        parent->recalculateVolume();
    }
  }

  struct CollisionProperties
  {
    float restitution;
  };

  class Collider
  {
  protected:
    Collider(Rigidbody &body, const Mat3 &transform) : body(&body), transform(transform), invTransform(transform.inverse()) {}

    // offset of the collider in object space
    Mat3 transform, invTransform;

  public:
    Rigidbody *body;

    // gets the origin in object space
    inline Vec2 origin() const { return transform.getColumn(2); }

    inline Mat3 getTransform() const { return transform; }
    inline Mat3 getInvTransform() const { return invTransform; }
    inline void setTransform(const Mat3 &transform)
    {
      this->transform = transform;
      invTransform = transform.inverse();
    }
    inline void setInvTransform(const Mat3 &invTarnsform)
    {
      this->invTransform = invTarnsform;
      transform = invTarnsform.inverse();
    }

    inline Vec2 collider2Object(const Vec2 &v) const { return transform * v; }
    inline Vec2 object2Collider(const Vec2 &v) const { return invTransform * v; }
    inline Vec2 rotCollider2Object(const Vec2 &v) const { return transform.getRotation() * v; }
    inline Vec2 rotObject2Collider(const Vec2 &v) const { return invTransform.getRotation() * v; }
  };

  class CircleCollider : public Collider
  {
  public:
    // center is based on offset
    float radius;

    CircleCollider(float radius, Rigidbody &body, const Mat3 &transform = Mat3::identity()) : radius(abs(radius)), Collider(body, transform) {}
  };

  class BoxCollider : public Collider
  {
  public:
    Vec2 halfSize;

    BoxCollider(const Vec2 &halfSize, Rigidbody &body, const Mat3 &transform = Mat3::identity()) : halfSize(abs(halfSize.x), abs(halfSize.y)), Collider(body, transform) {}
  };

  class IntersectionDetector
  {
  public:
    static bool circleCircle(const CircleCollider &c1, const CircleCollider &c2);
    static bool boxCircle(const BoxCollider &bc, const CircleCollider &cc);
    static bool boxBox(const BoxCollider &b1, const BoxCollider &b2);
  };

  // offers methods that check for collisions between colliders
  class CollisionDetector
  {
  public:
    static std::list<Contact> &circleCircle(const CircleCollider &c1, const CircleCollider &c2, const CollisionProperties &properties);
    static std::list<Contact> &boxCircle(const BoxCollider &bc, const CircleCollider &cc, const CollisionProperties &properties);
    static std::list<Contact> &boxBox(const BoxCollider &b1, const BoxCollider &b2, const CollisionProperties &properties);
  };

  // template class implementations

  template <typename BVClass>
  BVHNode<BVClass>::~BVHNode()
  {
    if (parent)
    {
      BVHNode *sibling;
      if (parent->children[0] == this)
        sibling = parent->children[1];
      else
        sibling = parent->children[0];

      parent->volume = sibling->volume;
      parent->body = sibling->body;
      parent->children[0] = sibling->children[0];
      parent->children[1] = sibling->children[1];

      sibling->parent = nullptr;
      sibling->body = nullptr;
      sibling->children[0] = nullptr;
      sibling->children[1] = nullptr;
      delete sibling;

      parent->recalculateBoundingVolume();
    }

    if (children[0])
    {
      children[0]->parent = nullptr;
      delete children[0];
    }
    if (children[1])
    {
      children[1]->parent = nullptr;
      delete children[1];
    }
  }

  template <typename BVClass>
  void BVHNode<BVClass>::insert(Rigidbody &body, BVClass &volume)
  {
    if (isLeaf())
    {
      children[0] = new BVHNode(this->volume, this->body);
      children[0]->parent = this;
      children[1] = new BVHNode(volume, &body);
      children[1]->parent = this;
      this->body = nullptr;
    }
    else
    {
      // add to the child such that there is the least amount of growth
      if (children[0]->volume.growth(volume) > children[1]->volume.growth(volume))
        children[1]->insert(body, volume);
      else
        children[0]->insert(body, volume);
    }
  }

  template <typename BVClass>
  std::list<PotentialContact *> &BVHNode<BVClass>::potentialContacts(BVHNode<BVClass> &n1, BVHNode<BVClass> &n2)
  {
    auto contacts = new std::list<PotentialContact *>();

    // do nothing if there is no contact between the two trees
    if (!n1.overlaps(n2))
      return *contacts;

    if (n1.isLeaf() && n2.isLeaf())
      contacts->push_back(new PotentialContact(n1.body, n2.body));
    else if (n2.isLeaf() || (!n1.isLeaf() && n1.volume.size() >= n2.volume.size()))
    {
      contacts->splice(contacts->end(), potentialContacts(*n1.children[0], n2));
      contacts->splice(contacts->end(), potentialContacts(*n1.children[1], n2));
    }
    else
    {
      contacts->splice(contacts->end(), potentialContacts(*n2.children[0], n1));
      contacts->splice(contacts->end(), potentialContacts(*n2.children[1], n1));
    }

    return *contacts;
  }

  template <typename BVClass>
  std::list<PotentialContact *> &BVHNode<BVClass>::getPotentialContacts() const
  {
    if (!isLeaf())
      return potentialContacts(children[0], children[1]);
    else
      return *(new std::list<PotentialContact *>);
  }
}

#endif // EPHYS_COLLISION_H
