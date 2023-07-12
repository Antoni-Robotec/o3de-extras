/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root
 * of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "FingerGripper.h"
#include "Utils.h"

#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/SerializeContext.h>

#include <AzFramework/Physics/PhysicsSystem.h>
#include <ROS2/Manipulation/JointsManipulationRequests.h>
#include <imgui/imgui.h>

namespace ROS2
{
    void FingerGripper::Activate()
    {
        m_grippingInProgress = false;
        m_initialised = false;
        m_ImGuiPosition = 0;
        m_stallingFor = 0;
        AZ::TickBus::Handler::BusConnect();
        ImGui::ImGuiUpdateListenerBus::Handler::BusConnect();
        GripperRequestBus::Handler::BusConnect(GetEntityId());
    }
    void FingerGripper::Deactivate()
    {
        AZ::TickBus::Handler::BusDisconnect();
        ImGui::ImGuiUpdateListenerBus::Handler::BusDisconnect();
        GripperRequestBus::Handler::BusDisconnect(GetEntityId());
    }

    void FingerGripper::Reflect(AZ::ReflectContext* context)
    {
        if (AZ::SerializeContext* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<FingerGripper, AZ::Component>()->Field("Epsilon", &FingerGripper::m_epsilon)->Version(1);

            if (AZ::EditContext* ec = serialize->GetEditContext())
            {
                ec->Class<FingerGripper>("FingerGripper", "FingerGripper")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "FingerGripper")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC("Game"))
                    ->Attribute(AZ::Edit::Attributes::Category, "ROS2")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &FingerGripper::m_epsilon,
                        "Epsilon",
                        "A small value for feedback purposes (how slow the gripper has to move to be stalling, how close to the goal to "
                        "have reached it).");
            }
        }
    }

    ManipulationJoints& FingerGripper::GetFingerJoints()
    {
        if (m_fingerJoints.empty())
        {
            JointsManipulationRequestBus::EventResult(m_fingerJoints, GetEntityId(), &JointsManipulationRequests::GetJoints);
        }
        return m_fingerJoints;
    }

    void FingerGripper::SetPosition(float position, float maxEffort)
    {
        // A check to make this work with moveit
        if (maxEffort == 0)
        {
            maxEffort = AZStd::numeric_limits<float>::infinity();
        }

        float targetPosition = position;
        for (auto& [jointName, jointInfo] : m_fingerJoints)
        {
            AZ::Outcome<void, AZStd::string> result;
            JointsManipulationRequestBus::EventResult(
                result, GetEntityId(), &JointsManipulationRequests::MoveJointToPosition, jointName, targetPosition);
            if (!result.IsSuccess())
            {
                AZ_Warning("FingerGripper", result, "Joint move cannot be realized: %s", result.GetError().c_str());
            }
        }

        float oneMaxEffort = maxEffort / m_fingerJoints.size();
        for (auto& [jointName, jointInfo] : m_fingerJoints)
        {
            AZ::Outcome<void, AZStd::string> result;
            JointsManipulationRequestBus::EventResult(
                result, GetEntityId(), &JointsManipulationRequests::SetMaxJointEffort, jointName, oneMaxEffort);
            if (!result.IsSuccess())
            {
                AZ_Warning("FingerGripper", result, "Setting a max force for a joint cannot be realized: %s", result.GetError().c_str());
            }
        }
    }

    AZ::Outcome<void, AZStd::string> FingerGripper::GripperCommand(float position, float maxEffort)
    {
        m_grippingInProgress = true;
        m_desiredPosition = position;
        m_maxEffort = maxEffort;
        m_stallingFor = 0;

        SetPosition(position, maxEffort);

        return AZ::Success();
    }

    AZ::Outcome<void, AZStd::string> FingerGripper::CancelGripperCommand()
    {
        m_grippingInProgress = false;
        SetPosition(0, 0);
        return AZ::Success();
    }

    float FingerGripper::GetGripperPosition() const
    {
        AZ::Outcome<JointsManipulationRequests::JointsPositionsMap, AZStd::string> result;
        JointsManipulationRequestBus::EventResult(result, GetEntityId(), &JointsManipulationRequests::GetAllJointsPositions);
        auto positions = result.GetValue();

        // Should be numerically stable, as they are supposed to all be non-negative
        float gripperPosition = 0;
        for (const auto& [jointName, position] : positions)
        {
            gripperPosition += position;
        }

        return gripperPosition / m_fingerJoints.size();
    }

    float FingerGripper::GetGripperEffort() const
    {
        AZ::Outcome<JointsManipulationRequests::JointsEffortsMap, AZStd::string> result;
        JointsManipulationRequestBus::EventResult(result, GetEntityId(), &JointsManipulationRequests::GetAllJointsEfforts);

        auto efforts = result.GetValue();

        float gripperEffort = 0;
        for (const auto& [jointName, effort] : efforts)
        {
            gripperEffort += effort;
        }

        return gripperEffort;
    }

    bool FingerGripper::IsGripperVelocity0() const
    {
        AZ::Outcome<JointsManipulationRequests::JointsVelocitiesMap, AZStd::string> result;
        JointsManipulationRequestBus::EventResult(result, GetEntityId(), &JointsManipulationRequests::GetAllJointsVelocities);

        auto velocities = result.GetValue();

        for (const auto& [jointName, velocity] : velocities)
        {
            if (abs(velocity) > m_epsilon)
            {
                return false;
            }
        }
        return true;
    }

    bool FingerGripper::IsGripperNotMoving() const
    {
        return m_stallingFor > 1;
    }

    bool FingerGripper::HasGripperReachedGoal() const
    {
        return !m_grippingInProgress || abs(GetGripperPosition() - m_desiredPosition) < m_epsilon;
    }

    void FingerGripper::OnImGuiUpdate()
    {
        ImGui::Begin("FingerGripperDebugger");

        ImGui::SliderFloat("Target Position", &m_ImGuiPosition, 0, 0.1);

        if (ImGui::Button("Execute Command "))
        {
            GripperCommand(m_ImGuiPosition, AZStd::numeric_limits<float>::infinity());
        }

        ImGui::End();
    }

    void FingerGripper::OnTick([[maybe_unused]] float delta, [[maybe_unused]] AZ::ScriptTimePoint timePoint)
    {
        // @TODO: Hacky, can probably be done better
        if (!m_initialised)
        {
            m_initialised = true;
            GetFingerJoints();
            SetPosition(0, 0);
        }

        if (IsGripperVelocity0()) {
            m_stallingFor += delta;
        }
        else {
            m_stallingFor = 0;
        }
    }
} // namespace ROS2