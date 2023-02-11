#ifndef EPHYS_COLLISION_H
#define EPHYS_COLLISION_H

#include "ephys/math.h"
#include "ephys/contacts.h"
#include "ephys/collider.h"

#include <list>

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

    BoundingCircle() {}
    BoundingCircle(const Vec2 &center, float radius) : center(center), radius(radius) {}
    BoundingCircle(const BoundingCircle &b1, const BoundingCircle &b2);
    BoundingCircle(const BoundingCircle &bc) : center(bc.center), radius(bc.radius) {}

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

  class Collider;
  class Rigidbody;

  // a pair of rigidbodies athat are potentially in contact
  struct PotentialContact
  {
    Rigidbody *bodies[2];

    PotentialContact(Rigidbody &b1, Rigidbody &b2) : bodies{&b1, &b2} {}

    bool operator<(const PotentialContact &other) const
    {
      size_t thisMinId, thisMaxId,
          otherMinId, otherMaxId;

      if (bodies[0]->id > bodies[1]->id)
      {
        thisMaxId = bodies[0]->id;
        thisMinId = bodies[1]->id;
      }
      else
      {
        thisMaxId = bodies[1]->id;
        thisMinId = bodies[0]->id;
      }

      if (other.bodies[0]->id > other.bodies[1]->id)
      {
        otherMaxId = other.bodies[0]->id;
        otherMinId = other.bodies[1]->id;
      }
      else
      {
        otherMaxId = other.bodies[1]->id;
        otherMinId = other.bodies[0]->id;
      }

      if (bodies[0]->id == other.bodies[0]->id)
        return bodies[1]->id < other.bodies[1]->id;
      return bodies[0]->id < other.bodies[0]->id;
    }
    inline bool operator==(const PotentialContact &other) const
    {
      return (bodies[0] == other.bodies[0] && bodies[1] == other.bodies[1]) ||
             (bodies[0] == other.bodies[1] && bodies[1] == other.bodies[0]);
    }
  };

  template <typename BVClass>
  class BVHNode
  {
    static_assert(std::is_base_of<BoundingVolume, BVClass>::value, "bounding volume class must be derived from ephys::BoundingVolume");

  protected:
    static std::list<PotentialContact> &potentialContacts(BVHNode<BVClass> &n1, BVHNode<BVClass> &n2);
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

    void insert(BVClass &volume, Rigidbody *body);

    std::list<PotentialContact> &getPotentialContacts() const;
  };

  struct CollisionProperties
  {
    float restitution;
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

  class CollisionContactGenerator : public ContactGenerator
  {
  protected:
    std::list<Rigidbody *> *bodies;

  public:
    CollisionContactGenerator() : bodies{nullptr} {}
    CollisionContactGenerator(std::list<Rigidbody *> &bodies) : bodies{&bodies} {}

    inline std::list<Rigidbody *> getBodies() const { return *bodies; }
    inline void setBodies(std::list<Rigidbody *> &bodies) { this->bodies = &bodies; }

    std::list<Contact> &generateContacts() const;
  };

  // template class implementations

  template <typename BVClass>
  void BVHNode<BVClass>::recalculateVolume()
  {
    if (!isLeaf())
    {
      volume = BVClass(children[0]->volume, children[1]->volume);

      if (parent)
        parent->recalculateVolume();
    }
  }

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

      parent->recalculateVolume();
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
  void BVHNode<BVClass>::insert(BVClass &volume, Rigidbody *body)
  {
    if (isLeaf())
    {
      children[0] = new BVHNode(this->volume, this->body);
      children[0]->parent = this;
      children[1] = new BVHNode(volume, body);
      children[1]->parent = this;
      this->body = nullptr;
    }
    else
    {
      // add to the child such that there is the least amount of growth
      if (children[0]->volume.growth(volume) > children[1]->volume.growth(volume))
        children[1]->insert(volume, body);
      else
        children[0]->insert(volume, body);
    }
    recalculateVolume();
  }

  template <typename BVClass>
  std::list<PotentialContact> &BVHNode<BVClass>::potentialContacts(BVHNode<BVClass> &n1, BVHNode<BVClass> &n2)
  {
    auto contacts = new std::list<PotentialContact>();

    if (!n1.isLeaf())
      contacts->splice(contacts->end(), potentialContacts(*n1.children[0], *n1.children[1]));
    if (!n2.isLeaf())
      contacts->splice(contacts->end(), potentialContacts(*n2.children[0], *n2.children[1]));

    // there are only contacts if the nodes intersect
    if (n1.overlaps(n2))
    {
      if (n1.isLeaf() && n2.isLeaf())
        contacts->push_back(PotentialContact(*n1.body, *n2.body));
      else
      {
        if (n2.isLeaf() || (!n1.isLeaf() && n1.volume.size() >= n2.volume.size()))
        {
          contacts->splice(contacts->end(), potentialContacts(*n1.children[0], n2));
          contacts->splice(contacts->end(), potentialContacts(*n1.children[1], n2));
        }
        else
        {
          contacts->splice(contacts->end(), potentialContacts(*n2.children[0], n1));
          contacts->splice(contacts->end(), potentialContacts(*n2.children[1], n1));
        }
      }
    }

    return *contacts;
  }

  template <typename BVClass>
  std::list<PotentialContact> &BVHNode<BVClass>::getPotentialContacts() const
  {
    if (!isLeaf())
      return BVHNode<BVClass>::potentialContacts(*children[0], *children[1]);
    else
      return *(new std::list<PotentialContact>);
  }
}

#endif // EPHYS_COLLISION_H
