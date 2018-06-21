//
// Copyright (c) 2008-2016 the Urho3D project.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.
//

#include <Urho3D/Core/Context.h>
#include <Urho3D/Physics/PhysicsEvents.h>
#include <Urho3D/Physics/PhysicsWorld.h>
#include <Urho3D/Physics/PhysicsUtils.h>
#include <Urho3D/Physics/RigidBody.h>
#include <Urho3D/Physics/CollisionShape.h>
#include <Urho3D/Scene/Scene.h>
#include <Urho3D/Scene/SceneEvents.h>
#include <SDL/SDL_log.h>

#include "MovingPlatform.h"
#include "Character.h"

#include <Urho3D/DebugNew.h>
//=============================================================================
//=============================================================================
MovingPlatform::MovingPlatform(Context* context)
    : LogicComponent(context)
    , platformState_(PLATFORM_STATE_START)
    , platformSpeed_(5.0f)
    , updateBodyOnPlatform_(true)
    , hasLanded_(false)
{
    SetUpdateEventMask(0);
}

MovingPlatform::~MovingPlatform()
{
}

void MovingPlatform::RegisterObject(Context* context)
{
    context->RegisterFactory<MovingPlatform>();
}

void MovingPlatform::Initialize(Node *platformNode, const Vector3 &finishPosition, bool updateBodyOnPlatform)
{
    // get other lift components
    platformNode_ = platformNode;
    platformVolumdNode_ = platformNode_->GetChild("PlatformVolume", true);

    assert( platformNode_ && platformVolumdNode_ && "missing nodes!" );

    platformRigidBody_  = platformNode_->GetComponent<RigidBody>();
    platformVolumeBody_ = platformVolumdNode_->GetComponent<RigidBody>();
    SubscribeToEvent(platformVolumdNode_, E_NODECOLLISIONSTART, URHO3D_HANDLER(MovingPlatform, HandleVolumeEntered));
    SubscribeToEvent(platformVolumdNode_, E_NODECOLLISIONEND, URHO3D_HANDLER(MovingPlatform, HandleVolumeExited));

    // positions
    initialPosition_   = platformNode_->GetWorldPosition();
    finishPosition_    = finishPosition;
    directionToFinish_ = (finishPosition_ - initialPosition_).Normalized();

    // state
    platformState_ = PLATFORM_STATE_MOVETO_FINISH;
    updateBodyOnPlatform_ = updateBodyOnPlatform;
    prevPlatformPosition_ = initialPosition_;
    prevPlatformLinVelocity_ = Vector3::ZERO;

    SetUpdateEventMask(USE_FIXEDUPDATE);
}

void MovingPlatform::FixedUpdate(float timeStep)
{
    Vector3 platformPos = platformRigidBody_->GetPosition();
    Vector3 startingLinVel = platformRigidBody_->GetLinearVelocity();
    float curLinVel = startingLinVel.Length();
    float newLinVel = (curLinVel > platformSpeed_)?platformSpeed_:Lerp(curLinVel, platformSpeed_, 0.02f);

    // move platform
    if (platformState_ == PLATFORM_STATE_MOVETO_FINISH)
    {
        Vector3 curDistance  = finishPosition_ - platformPos;
        Vector3 curDirection = curDistance.Normalized();
        float dist = curDistance.Length();
        float dotd = directionToFinish_.DotProduct(curDirection);

        if (dotd > 0.0f)
        {
            // slow down near the end
            if (dist < 1.0f)
            {
                newLinVel = Lerp(newLinVel, 0.5f, 1.0f - dist);
            }

            Vector3 nlinVel = directionToFinish_ * newLinVel;
            platformRigidBody_->SetLinearVelocity(nlinVel);
        }
        else
        {
            platformRigidBody_->SetPosition(finishPosition_);
            platformRigidBody_->SetLinearVelocity(Vector3::ZERO);
            platformState_ = PLATFORM_STATE_MOVETO_START;
        }
    }
    else if (platformState_ == PLATFORM_STATE_MOVETO_START)
    {
        Vector3 curDistance  = initialPosition_ - platformPos;
        Vector3 curDirection = curDistance.Normalized();
        float dist = curDistance.Length();
        float dotd = directionToFinish_.DotProduct(curDirection);

        if (dotd < 0.0f)
        {
            // slow down near the end
            if (dist < 1.0f)
            {
                newLinVel = Lerp(newLinVel, 0.5f, 1.0f - dist);
            }

            Vector3 nlinVel = -directionToFinish_ * newLinVel;
            platformRigidBody_->SetLinearVelocity(nlinVel);
        }
        else
        {
            platformRigidBody_->SetPosition(initialPosition_);
            platformRigidBody_->SetLinearVelocity(Vector3::ZERO);
            platformState_ = PLATFORM_STATE_MOVETO_FINISH;
        }
    }

    // update body's movement
    UpdateBodyOnPlatform(timeStep);
}

void MovingPlatform::UpdateBodyOnPlatform(float timeStep)
{
    if (updateBodyOnPlatform_ && bodyOnPlatform_)
    {
        Vector3 platLinVel = platformRigidBody_->GetLinearVelocity();
        Character *character = bodyOnPlatform_->GetNode()->GetComponent<Character>();

        // mark as landed once it hits the platform
        if (character->IsOnGround())
        {
            hasLanded_ = true;
        }

        // add a slight braking to prevent body from sliding off immediately after landing
        if (!character->IsOnGround())
        {
            if (!hasLanded_)
            {
                Vector3 diffBodyVel = platLinVel - bodyOnPlatform_->GetLinearVelocity();
                diffBodyVel.y_ = 0.0f; // ignore vertical velocity
                diffBodyVel *= 0.004f;
                bodyOnPlatform_->ApplyImpulse(diffBodyVel);
            }
        }

        // apply change in platform velocity
        if (!character->IsJumping())
        {
            Vector3 deltaLinVel = platLinVel - prevPlatformLinVelocity_;

            if (deltaLinVel.Length() > 0.1f)
            {
                bodyOnPlatform_->ApplyImpulse(deltaLinVel);
            }
        }
    }

    // update prev
    prevPlatformPosition_ = platformRigidBody_->GetPosition();
    prevPlatformLinVelocity_ = platformRigidBody_->GetLinearVelocity();
}

void MovingPlatform::HandleVolumeEntered(StringHash eventType, VariantMap& eventData)
{
    using namespace NodeCollision;
    bodyOnPlatform_ = (RigidBody*)eventData[P_OTHERBODY].GetVoidPtr();

    // verify it's a character
    if (bodyOnPlatform_->GetNode()->GetComponent<Character>())
    {
        bodyOnPlatform_->GetNode()->GetComponent<Character>()->SetOnMovingPlatform(platformRigidBody_);

        // set
        prevPlatformPosition_ = platformRigidBody_->GetPosition();
        hasLanded_ = false;
    }
    else
    {
        bodyOnPlatform_ = NULL;
    }
}

void MovingPlatform::HandleVolumeExited(StringHash eventType, VariantMap& eventData)
{
    using namespace NodeCollision;

    if (bodyOnPlatform_->GetNode()->GetComponent<Character>())
    {
        // 2.5x force (see FORCE_MULTIPLYER_ON_PLATFORM in character.cpp) is added 
        // when moving on a platform, half the body's horizontal velocity as it exits
        Vector3 linVel = bodyOnPlatform_->GetLinearVelocity();
        linVel.x_ *= 0.5f;
        linVel.z_ *= 0.5f;
        bodyOnPlatform_->SetLinearVelocity(linVel);

        bodyOnPlatform_->GetNode()->GetComponent<Character>()->SetOnMovingPlatform(NULL);
    }

    // clear
    bodyOnPlatform_ = NULL;
    hasLanded_ = false;
}



