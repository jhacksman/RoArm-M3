# Leader-Follower Mode Research

This document serves as a placeholder for comprehensive research on the Leader-Follower mode functionality for the RoArm-M3 Pro. This is identified as the highest priority research area based on user requirements.

## What is Leader-Follower Mode?

Leader-Follower mode (also known as "teaching mode" or "imitation learning") is a control method where one robotic arm (the leader) is manually positioned, and another robotic arm (the follower) mimics these movements in real-time or records them for later playback.

## Research Goals

1. Identify if the RoArm-M3 Pro officially supports Leader-Follower mode
2. Document the setup process for enabling this functionality
3. Collect code examples and implementation details
4. Translate any Chinese-language resources related to this functionality
5. Provide a step-by-step guide for implementing Leader-Follower mode

## Current Status

This is a placeholder document. The research on Leader-Follower mode is ongoing, with the following approaches being pursued:

1. Examining official Waveshare documentation
2. Searching for community implementations
3. Investigating the LeRobot AI project integration which may include imitation learning capabilities
4. Searching Chinese-language resources for additional information
5. Analyzing the JSON command system for relevant commands

## Preliminary Findings

Based on initial research from the Waveshare wiki:

1. The RoArm-M3 has a "DEFA: Dynamic External Force Adaptive Control" feature that may be related to manual positioning
2. The arm supports the LeRobot AI project which includes "imitation learning" capabilities
3. The "Torque: Torque Lock Control" feature allows manual rotation of joints when disabled

These features may be building blocks for implementing a Leader-Follower mode, but further research is needed to confirm official support and implementation details.

## Next Steps

1. Download and analyze all available software resources
2. Search for specific JSON commands related to position recording and playback
3. Investigate the LeRobot AI project for imitation learning capabilities
4. Search Chinese-language forums and documentation
5. Compile findings into a comprehensive guide

This document will be updated as research progresses.
