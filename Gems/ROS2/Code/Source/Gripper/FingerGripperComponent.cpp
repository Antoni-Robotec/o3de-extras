/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root
 * of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "FingerGripperComponent.h"
#include "Utils.h"

#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/SerializeContext.h>

#include <AzFramework/Physics/PhysicsSystem.h>
#include <ROS2/Manipulation/JointsManipulationRequests.h>
#include <imgui/imgui.h>

namespace ROS2
{
    void FingerGripperComponent::Activate()
    {
        m_grippingInProgress = false;
        m_initialised = false;
        m_ImGuiPosition = 0.0f;
        m_stallingFor = 0.0f;
        AZ::TickBus::Handler::BusConnect();
        ImGui::ImGuiUpdateListenerBus::Handler::BusConnect();
        GripperRequestBus::Handler::BusConnect(GetEntityId());
    }
    void FingerGripperComponent::Deactivate()
    {
        AZ::TickBus::Handler::BusDisconnect();
        ImGui::ImGuiUpdateListenerBus::Handler::BusDisconnect();
        GripperRequestBus::Handler::BusDisconnect(GetEntityId());
    }

    void FingerGripperComponent::Reflect(AZ::ReflectContext* context)
    {
        if (AZ::SerializeContext* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<FingerGripperComponent, AZ::Component>()->Field("Epsilon", &FingerGripperComponent::m_epsilon)->Version(1);

            if (AZ::EditContext* ec = serialize->GetEditContext())
            {
                ec->Class<FingerGripperComponent>("FingerGripperComponent", "Component controlling a finger gripper.")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "FingerGripperComponent")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC("Game"))
                    ->Attribute(AZ::Edit::Attributes::Category, "ROS2")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &FingerGripperComponent::m_epsilon,
                        "Epsilon",
                        "A small value for feedback purposes (how slow the gripper has to move to be stalling, how close to the goal to "
                        "have reached it).");
            }
        }
    }

    void FingerGripperComponent::GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        required.push_back(AZ_CRC_CE("JointsManipulationService"));
    }

    ManipulationJoints& FingerGripperComponent::GetFingerJoints()
    {
        if (m_fingerJoints.empty())
        {
            JointsManipulationRequestBus::EventResult(m_fingerJoints, GetEntityId(), &JointsManipulationRequests::GetJoints);
        }
        return m_fingerJoints;
    }

    void FingerGripperComponent::SetPosition(float position, float maxEffort)
    {
        // A check to make this work with moveit
        if (maxEffort == 0.0f)
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
                AZ_Warning("FingerGripperComponent", result, "Joint move cannot be realized: %s", result.GetError().c_str());
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
                AZ_Warning("FingerGripperComponent", result, "Setting a max force for a joint cannot be realized: %s", result.GetError().c_str());
            }
        }
    }

    AZ::Outcome<void, AZStd::string> FingerGripperComponent::GripperCommand(float position, float maxEffort)
    {
        m_grippingInProgress = true;
        m_desiredPosition = position;
        m_maxEffort = maxEffort;
        m_stallingFor = 0.0f;

        SetPosition(position, maxEffort);

        return AZ::Success();
    }

    AZ::Outcome<void, AZStd::string> FingerGripperComponent::CancelGripperCommand()
    {
        m_grippingInProgress = false;
        SetPosition(0.0f, 0.0f);
        return AZ::Success();
    }

    float FingerGripperComponent::GetGripperPosition() const
    {
        AZ::Outcome<JointsManipulationRequests::JointsPositionsMap, AZStd::string> result;
        JointsManipulationRequestBus::EventResult(result, GetEntityId(), &JointsManipulationRequests::GetAllJointsPositions);
        auto positions = result.GetValue();

        // Should be numerically stable, as they are supposed to all be non-negative
        float gripperPosition = 0.0f;
        for (const auto& [jointName, position] : positions)
        {
            gripperPosition += position;
        }

        return gripperPosition / m_fingerJoints.size();
    }

    float FingerGripperComponent::GetGripperEffort() const
    {
        AZ::Outcome<JointsManipulationRequests::JointsEffortsMap, AZStd::string> result;
        JointsManipulationRequestBus::EventResult(result, GetEntityId(), &JointsManipulationRequests::GetAllJointsEfforts);

        auto efforts = result.GetValue();

        float gripperEffort = 0.0f;
        for (const auto& [jointName, effort] : efforts)
        {
            gripperEffort += effort;
        }

        return gripperEffort;
    }

    bool FingerGripperComponent::IsGripperVelocity0() const
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

    bool FingerGripperComponent::IsGripperNotMoving() const
    {
        return m_stallingFor > 1.0f;
    }

    bool FingerGripperComponent::HasGripperReachedGoal() const
    {
        return !m_grippingInProgress || abs(GetGripperPosition() - m_desiredPosition) < m_epsilon;
    }

    void FingerGripperComponent::OnImGuiUpdate()
    {
        ImGui::Begin("FingerGripperDebugger");

        ImGui::SliderFloat("Target Position", &m_ImGuiPosition, 0.0f, 0.1f);

        if (ImGui::Button("Execute Command"))
        {
            GripperCommand(m_ImGuiPosition, AZStd::numeric_limits<float>::infinity());
        }

        ImGui::End();
    }

    void FingerGripperComponent::OnTick([[maybe_unused]] float delta, [[maybe_unused]] AZ::ScriptTimePoint timePoint)
    {
        // @TODO: Hacky, can probably be done better
        if (!m_initialised)
        {
            m_initialised = true;
            GetFingerJoints();
            SetPosition(0.0f, 0.0f);
        }

        if (IsGripperVelocity0()) {
            m_stallingFor += delta;
        }
        else {
            m_stallingFor = 0.0f;
        }
    }
} // namespace ROS2