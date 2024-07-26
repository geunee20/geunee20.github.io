---
layout: post
title: "[Controller Design] Day 0: Kicking Off the Controller Design Project"
date: 2024-05-14 11:31:00-0400
description: An overview of project exploring controller design techniques.
tags:
categories: Study_with_Me Controller_Design
disqus_comments: true
related_posts: true
---

**This project was inspired by the following video:**

<div style="position: relative; padding-bottom: 56.25%; height: 0; overflow: hidden; margin-bottom: 3rem;">
  <iframe src="https://www.youtube.com/embed/I5GvwWKkBmg" frameborder="0" style="position: absolute; top: 0; left: 0; width: 100%; height: 100%;" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>
</div>

## Journey Through the Dynamics of multi-link mechanisms: A Project Exploration

The inaugural step of this journey begins with constructing the multibody system for the multi-link mechanism. This stage is crucial as it sets the groundwork for all subsequent experiments. I am designing multi-link mechanisms with varying numbers of links, each introducing unique challenges and dynamics. By setting up the initial simulation environment and examining how the system behaves under different initial conditions, I aim to gather a robust understanding of the fundamental dynamics that govern these intriguing mechanical structures.

#### Phase 1: Building the Foundation

The first phase involves building a multibody system for the multi-link mechanism. This foundational step includes designing multi-link mechanisms with different numbers of links and setting up the initial simulation environment. By understanding the basic dynamics and how the system responds under various initial conditions, I can prepare for more complex control implementations.

#### Phase 2: Achieving Stability with PID and LQR Control

Once the basic system is operational, I focus on stabilizing a multi-link mechanism. Initially, a PID controller is applied to the joints to achieve stable, controlled motion. Tuning the PID parameters is crucial for maintaining the mechanism's upright position despite disturbances. I also integrate a Linear Quadratic Regulator (LQR) to complement the PID controller. LQR optimally minimizes a cost function that balances state deviation and control effort, enhancing stability and robustness under linear conditions. The combination of PID and LQR establishes a robust control foundation, efficiently managing disturbances and laying the groundwork for more dynamic challenges in subsequent phases.

#### Phase 3: Calculating the Equations of Motion for Non-linear Controller Design

Building on the foundational understanding and adaptations to a dynamic base, this phase focuses on calculating the equations of motion for the multi-link mechanism. I will use Lagrangian mechanics to derive precise mathematical models that describe the system’s dynamics, essential for predicting its behavior under various conditions. These equations will directly inform the design of advanced nonlinear controllers, enabling optimized control strategies and improved system performance in subsequent phases.

#### Phase 4: Advanced Nonlinear Control Strategies

In the final phases of our project, I integrate several advanced nonlinear control strategies to handle our moving base multi-link mechanism, particularly focusing on robust and intelligent methodologies.

- **PID with Gravity Compensation:** Enhances basic PID control by incorporating compensation for gravitational forces acting on the system, crucial for maintaining stability and control in vertically oriented mechanisms.
- **PID with Computed Torque Method:** Combines conventional PID control with a model-based torque computation that accounts for the dynamics of the system, significantly improving precision and response to disturbances.
- **Bang-Bang Controller:** Offers a simple yet effective control strategy for rapid switching between maximum states, useful for systems requiring stark transitions.
- **Fuzzy Logic Controller:** Introduces an ability to handle uncertainties and imprecise inputs, making decisions based on 'degrees of truth' rather than binary logic, ideal for complex, nonlinear system behaviors.
- **Gain Scheduling Controller:** Adjusts controller gains based on real-time changes in system parameters or operating conditions, enhancing control flexibility and responsiveness.
- **Self-Tuning Controller:** Adapts its parameters in real-time based on system feedback, ensuring optimal performance under varying conditions.
- **Model-Reference Adaptive Control (MRAC):** Aims to modify the system dynamics to follow a desired reference model, useful for ensuring stability in dynamically changing environments.
- **Model Predictive Control (MPC):** Utilizes a model of the system to predict future states and optimize control actions over a set horizon, providing a strategic depth to immediate and future control needs.
- **Sliding Mode Control (SMC):** Features a discontinuous control law ensuring robustness against model uncertainties and external disturbances, ideal for systems where precise control is crucial despite parameter variations.
- **Backstepping Controller:** Utilizes a recursive design approach that builds a Lyapunov function step-by-step, making it suitable for systems with strict stability requirements.
- **Neural Network Controller:** Employs artificial neural networks to model complex nonlinearities and dynamic behaviors, enabling adaptive control strategies tailored to specific operational profiles.

These control strategies collectively aim to address the multi-faceted challenges presented by the multi-link mechanism, ensuring robustness, adaptability, and high performance in real-world operational scenarios.

<br>
<br>

Each phase of this project is designed to not only build on the previous experiences but also to introduce and overcome new complexities. This systematic escalation ensures a thorough exploration and understanding of each control strategy implemented.

I invite you to stay tuned as I document this exciting journey, sharing detailed insights and results from each phase of my exploration into the dynamic realm of multi-link mechanisms. This blog will serve as a chronicle of my learning, challenges, breakthroughs, and, ultimately, the evolution of my understanding of control dynamics. Join me on this adventure as I push the limits of technology and engineering!

<br>
<br>
<br>

##### You can find the Simulink model on my [GitHub repository](https://github.com/geunee20/Controller_Design).
