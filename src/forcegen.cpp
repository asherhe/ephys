#include "ephys/forcegen.h"
#include "ephys/rigidbody.h"

using namespace ephys;

void ForceRegistry::add(ForceGenerator &fgen, Rigidbody &body)
{
  ForcePair *pair = new ForcePair;
  pair->body = &body;
  pair->fgen = &fgen;
  reg.push_back(*pair);
}
void ForceRegistry::remove(ForceGenerator &fgen, Rigidbody &body)
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
  Vec2 anchorWS = body.local2World(anchor), endAnchorWS = end->local2World(endAnchor);

  Vec2 displacement = endAnchorWS - anchorWS;
  float distance = displacement.norm();
  // anchors coincide, so the spring has no direction to act along
  if (distance < 1e-6f)
    return;
  Vec2 dir = displacement / distance;

  // velocities of the anchor points: v + w x r, with r the anchor's offset from the center in world space
  Vec2 r = body.rotLocal2World(anchor), endR = end->rotLocal2World(endAnchor);
  Vec2 anchorVel = body.getVel() + Vec2(-r.y, r.x) * body.getAngVel(),
       endAnchorVel = end->getVel() + Vec2(-endR.y, endR.x) * end->getAngVel();
  // positive when the anchors are separating
  float separationSpeed = (endAnchorVel - anchorVel) * dir;

  Vec2 force = dir * (k * (distance - length) + damping * separationSpeed);

  body.addForceAtLocal(force, anchor);
  end->addForceAtLocal(-force, endAnchor);
}
