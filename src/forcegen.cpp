#include "ephys/forcegen.h"
#include "ephys/rigidbody.h"

using namespace ephys;

void ForceRegistry::add(Rigidbody &body, ForceGenerator &fgen)
{
  ForcePair *pair = new ForcePair;
  pair->body = &body;
  pair->fgen = &fgen;
  reg.push_back(*pair);
}
void ForceRegistry::remove(Rigidbody &body, ForceGenerator &fgen)
{
  for (auto it = reg.begin(); it != reg.end(); ++it)
    if (it->body == &body && it->fgen == &fgen)
    {
      reg.erase(it);
      return;
    }
}
void ForceRegistry::clear()
{
  reg.clear();
}
void ForceRegistry::step(float dt)
{
  for (auto it = reg.begin(); it != reg.end(); ++it)
    it->fgen->updateForce(*(it->body), dt);
}

void Gravity::updateForce(Rigidbody &body, float dt)
{
  body.addForce(body.getMass() * gravity);
}

void Spring::updateForce(Rigidbody &body, float dt)
{
  // get anchors in world space
  Vec2 anchorWS = body.local2World(anchor), endAnchorWS = end.local2World(anchor);

  Vec2 displacement = endAnchorWS - anchorWS;
  float restDisplacement = displacement.norm() - length;
  body.addForceAtLocal(displacement.normalize() * k * restDisplacement, anchor);
}
