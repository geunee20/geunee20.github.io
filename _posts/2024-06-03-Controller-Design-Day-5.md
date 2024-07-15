---
layout: post
title: "[Controller Design] Day 5 - Fully Actuated Multi-Link Mechanism / PID Control with Computed Torque Method"
date: 2024-06-03 00:00:00-0400
description: Refining multi-link mechanism control using PID techniques combined with the computed torque method for improved precision and dynamic stability.
tags: Simulink Control Controller_Design
categories: Study_with_Me
disqus_comments: true
related_posts: true
thumbnail: assets/img/posts/2024-06-03-Controller-Design-Day-5/00_thumbnail.gif
images:
  compare: true
  slider: true
---

# I. Introduction

On Day 4, I realized that while a PID controller with gravity compensation stabilizes the system faster, it often struggles with overreactions and potential destabilizations. This occurs because it focuses solely on angular positions, neglecting angular velocities.

Building on the equations of motion derived on Day 3 for the triple-link mechanism:

$$
\tau = M(q)\ddot{q} + h(q, \dot{q}) + g(q) \tag{1}
$$

Where:

- **Matrix M (Inertial Term)**: Reflects the Newtonian response to applied forces, considering the inertia of the masses.
- **Matrix h (Coriolis and Centrifugal Forces)**: Accounts for forces due to system velocities and rotational dynamics.
- **Matrix g (Gravitational Term)**: Represents the gravitational forces acting on each segment.

Today, I introduce a nonlinear control system that combines a PID controller with the computed torque method. This system tackles the nonlinear dynamics effectively, compensating for gravity and dynamic forces while integrating displacement and velocity feedback. This approach, known as feedback linearization, linearizes the control by canceling out nonlinearities, enhancing system stability and tracking accuracy.

The updated methodology reduces the need for extensive parameter tuning and improves response times, making the system more reliable and user-friendly. It is particularly suited for advanced robotics and precision engineering applications where accuracy and dynamic performance are crucial.

The revised block diagrams in Figures 1 and 2 illustrate how the computed torque method enhances the PID controller by leveraging the system's full dynamics to accurately predict and correct disturbances. These diagrams are referenced from Dr. Jun Ueda's lecture slides.

1. **Linearized Dynamics:**
   The linearized system dynamics are introduced using the equation:

   $$
   \tau = M(q)\ddot{q}' + h(q, \dot{q}) + g(q) \tag{2}
   $$

   This formula for $$\tau$$, based on the computed input $$\ddot{q}'$$, quantifies the total force required to achieve the control objectives. The dynamics without control input adjustment, given by Equation 1, are aligned with this formulation to yield:

   $$
   M(q)\ddot{q}' + h(q, \dot{q}) + g(q) = M(q)\ddot{q} + h(q, \dot{q}) + g(q) \Rightarrow \ddot{q}' = \ddot{q} \tag{3}
   $$

   This result indicates that under ideal control conditions, where the system accurately follows the control input, $$\ddot{q}'$$ aligns with the actual acceleration $$\ddot{q}$$ needed to match the desired trajectory, thereby confirming that the new control input effectively compensates for all dynamic forces impacting the system.

2. **Input Through PID Controller:**
   The new input $$\ddot{q}'$$ is formulated using a PID controller to correct the error, described by the equation:

   $$
   \ddot{q}' = \ddot{q}_d + K_d(\dot{q}_d - \dot{q}) + K_p(q_d - q) + K_i\int{(q_d - q)}dt \tag{4}
   $$

   Using Equations 3 and 4, the dynamics correction can be expressed as:

   $$
   (\ddot{q}_d - \ddot{q}) + K_d(\dot{q}_d - \dot{q}) + K_p(q_d - q) + K_i\int{(q_d - q)}dt \tag{5}
   $$

3. **Define the Error Dynamics:**
   The tracking error is foundational for deriving the system's dynamics, defined by:

   $$
   e = q_d - q \tag{6}
   $$

   This definition facilitates the expression of derivatives in terms of error:

   $$
   \dot{e} = \dot{q}_d - \dot{q}, \quad \ddot{e} = \ddot{q}_d - \ddot{q} \tag{7}
   $$

4. **Error Dynamics Equation:**
   Derived from Equation 5 by substituting Equations 6 and 7, the full error dynamics is given by:

   $$
   \ddot{e} + K_d\dot{e} + K_pe + K_i \int{e}dt = 0 \tag{9}
   $$

   This equation establishes the control dynamics under the implemented inputs, serving as a crucial indicator for system stability.

5. **Stability Analysis:**
   The stability of the system is evaluated using the [Routh–Hurwitz criterion](https://en.wikipedia.org/wiki/Routh%E2%80%93Hurwitz_stability_criterion), which is a well-established method for determining the stability of linear control systems. According to this criterion, the system is considered stable if:

   $$
   K_dK_p > K_i \tag{8}
   $$

   This inequality must hold to ensure that the system approaches stability, meaning that as time approaches infinity, the error effectively reduces to zero:

   $$
   t \rightarrow \infty \Rightarrow e \rightarrow 0.
   $$

# II. Modeling

### 1. Overall System

The overall system is illustrated in Figure 3. As observed, the system architecture bears resemblance to a traditional PID control setup but incorporates additional subsystems and components to enhance functionality. These new elements are the "MUX (Red box)" and "DEMUX (Blue box)", which play critical roles in handling signal matrices within the control loop.

- **MUX (Red box)**: The Multiplexer (MUX) is used to combine multiple single signals into a single matrix. This is particularly useful in control systems like ours where multiple input signals need to be processed simultaneously in a compact form. In Simulink, a MUX block simplifies system connections by aggregating several input lines into one output, feeding into systems that require a single input stream containing multiple signals.

- **DEMUX (Blue box)**: Conversely, the Demultiplexer (DEMUX) is used to split a matrix into its component single signals. This is essential for systems that process signals in a matrix form but then require individual signal handling downstream. In Simulink, the DEMUX block facilitates the distribution of a single input containing multiple data points into multiple outputs, which can then be directed to specific parts of the system.

By channeling matrices through a "Computed Torque" subsystem using these blocks, the system is able to compute and output a new feedback torque that is more accurately aligned with the dynamic requirements of the controlled process.

<div class="col-sm mt-3 mt-md-0">
    {% include figure.liquid loading="eager" path="assets/img/posts/2024-06-03-Controller-Design-Day-5/03_overall_system.png" class="img-fluid rounded z-depth-1" zoomable=true %}
</div>
<div class="caption">
    Figure 3. Schematic of the Multi-Link Mechanism Control System.
</div>

### 2. Computed Torque Method Block

Calculating torque through circuit design can be complex and time-consuming. Similar to the gravity calculation block described on Day 4, the computed torque method block simplifies this process. As depicted in Figure 4 and detailed through MATLAB scripts in Figures 5-7, I utilize "MATLAB Function blocks" within the computed torque block. These blocks allow me to script MATLAB code directly to compute each term of Equation 1. It’s important to note that I used a Matrix multiplication block instead of an element-wise multiplication block because Equation 1 involves matrix calculations, necessitating proper alignment and dimension matching of matrices.

- **$$M(q)$$**: This block computes the mass matrix (M) from Equation 1, which is critical as it reflects the system's inertia. The matrix $$M$$ is derived by partially differentiating Equation 1 with respect to each angular acceleration.

- **$$h(q, \dot{q})$$**: This block calculates the Coriolis and centrifugal forces, represented as the second term in Equation 1. These dynamic forces are crucial for accurately modeling the behavior of rotating systems. The matrix $$h$$ is obtained by isolating these forces in Equation 1, which can be done by subtracting $$M(q)\ddot{q} + g(q)$$ from the total torque $$\tau$$.

- **$$g(q)$$**: This block computes the gravitational forces, consistent with the gravity block used on Day 4. This ensures that gravitational effects are accurately integrated into the torque computation. The matrix $$g$$ is derived by isolating the gravitational term in Equation 1, which involves differentiating the gravitational components and then reintegrating them into the model.

<div style="width: 60%; margin: 0 auto;">
<div class="col-sm mt-3 mt-md-0">
    {% include figure.liquid loading="eager" path="assets/img/posts/2024-06-03-Controller-Design-Day-5/04_computed_torque_method.png" class="img-fluid rounded z-depth-1" %}
</div>
</div>
<div class="caption">
    Figure 4. Computed Torque Method Block, illustrating the integration of MATLAB Function blocks for computing torque components.
</div>

<div style="width: 80%; margin: 0 auto;">
<swiper-container keyboard="true" navigation="true" pagination="true" pagination-clickable="true" pagination-dynamic-bullets="true" rewind="true">
  <swiper-slide>{% include figure.liquid loading="eager" path="assets/img/posts/2024-06-03-Controller-Design-Day-5/05_block_M.png" class="img-fluid rounded z-depth-1"%}</swiper-slide>
  <swiper-slide>{% include figure.liquid loading="eager" path="assets/img/posts/2024-06-03-Controller-Design-Day-5/06_block_h.png" class="img-fluid rounded z-depth-1"%}</swiper-slide>
  <swiper-slide>{% include figure.liquid loading="eager" path="assets/img/posts/2024-06-03-Controller-Design-Day-5/07_block_g.png" class="img-fluid rounded z-depth-1"%}</swiper-slide>
</swiper-container>
</div>
<div class="caption">
    Figures 5 - 7. MATLAB Script for MATLAB Function Blocks for M, h, and g matrices.
</div>

It's important to note that $$q1$$ has been defined as $$q(2) + \frac{\pi}{2}$$. This definition is used because $$\theta_1$$ is set as $$\theta_1 = \frac{\pi}{2} + q_1$$ on Day 3.

# III. Simulation

In this section, I explore the simulation, focusing on both tracking and regulation capabilities of the multi-link mechanism control system. Through these simulations, I aim to validate the effectiveness of the controllers in maintaining trajectory accuracy (tracking) and stabilizing the system against disturbances (regulation).

### 1. Tracking

##### a) No PID Input

Initially, I set all the gains of the PID controller to zero to evaluate whether the computed torque block alone could effectively track the reference input. The results indicated that without the PID gains, the system failed to track the reference input and could not stabilize. I extended the simulation to 10,000 seconds, but the system remained unstable throughout, exhibiting behavior akin to an upside-down, slowly oscillating triple-link mechanism, which also showed signs of chaotic dynamics. This instability likely stems from the absence of error correction typically provided by the PID controller. The simulation video and the graph depicting each joint's parameters are presented in Video 1 and Figure 8, respectively.

<div style="width: 80%; margin: 0 auto;">
    <div class="col-sm mt-3 mt-md-0">
        {% include video.liquid path="assets/video/posts/2024-06-03-Controller-Design-Day-5/01_no_PID_input.mp4" class="img-fluid rounded z-depth-1" controls=true muted=true %}
    </div>
</div>
<div class="caption">
    Video 1. Tracking of the Triple-Link Mechanism without PID input.
</div>

<div style="width: 80%; margin: 0 auto;">
<div class="col-sm mt-3 mt-md-0">
    {% include figure.liquid loading="eager" path="assets/img/posts/2024-06-03-Controller-Design-Day-5/08_No_PID.png" class="img-fluid rounded z-depth-1" zoomable=true %}
</div>
</div>
<div class="caption">
    Figure 8. Graphs of Joint Parameters without PID input.
</div>

##### b) Varying PID gains

###### 1) $$K_p = 0.1, K_i = 0.001, K_d = 0.01$$

I began with very small PID controller gains, anticipating that the results would be similar to those without PID input. However, the outcome was entirely different from what I had expected. The system rotated counterclockwise and was not fully chaotic. While Base 1, Link 1, and Link 3 became marginally stable, Link 2 remained unstable. Video 2 and Figure 9 depict the initial phase of the triple-link mechanism, while Video 3 and Figure 10 showcase the middle phase.

<div class="row mt-3">
    <div class="col-sm mt-3 mt-md-0">
        {% include video.liquid path="assets/video/posts/2024-06-03-Controller-Design-Day-5/02_tracking_1.mp4" class="img-fluid rounded z-depth-1" controls=true muted=true %}
    </div>
    <div class="col-sm mt-3 mt-md-0">
        {% include video.liquid path="assets/video/posts/2024-06-03-Controller-Design-Day-5/03_tracking_1_1.mp4" class="img-fluid rounded z-depth-1" controls=true muted=true %}
    </div>
</div>
<div class="caption">
    Video 2 - 3. Tracking of the Triple-Link Mechanism with \( K_p = 0.1, K_i = 0.001, K_d = 0.01 \).
</div>

<div style="width: 80%; margin: 0 auto;">
<swiper-container keyboard="true" navigation="true" pagination="true" pagination-clickable="true" pagination-dynamic-bullets="true" rewind="true">
  <swiper-slide>{% include figure.liquid loading="eager" path="assets/img/posts/2024-06-03-Controller-Design-Day-5/09_tracking_1.png" class="img-fluid rounded z-depth-1" zoomable=true %}</swiper-slide>
  <swiper-slide>{% include figure.liquid loading="eager" path="assets/img/posts/2024-06-03-Controller-Design-Day-5/10_tracking_1_1.png" class="img-fluid rounded z-depth-1" zoomable=true %}</swiper-slide>
</swiper-container>
</div>
<div class="caption">
    Figure 9 - 10. Graphs of Joint Parameters with \( K_p = 0.1, K_i = 0.001, K_d = 0.01 \).
</div>

###### 2) $$K_p = 1, K_i = 0.01, K_d = 0.1$$

When I increased the gains slightly, the multi-link mechanism stabilized after oscillating for an extended period. An interesting observation was the lag in oscillation between the links. The maximum amplitude of Link 1 occurs first, followed by Link 2, and then Link 3. Also, intriguingly, when all the links are oscillating, the motion resembles that of a tail. Video 4 and Figure 11 display the results of the simulation.

<div style="width: 80%; margin: 0 auto;">
    <div class="col-sm mt-3 mt-md-0">
        {% include video.liquid path="assets/video/posts/2024-06-03-Controller-Design-Day-5/04_tracking_2.mp4" class="img-fluid rounded z-depth-1" controls=true muted=true %}
    </div>
</div>
<div class="caption">
    Video 4. Tracking of the Triple-Link Mechanism with \( K_p = 1, K_i = 0.01, K_d = 0.1 \).
</div>

<div style="width: 80%; margin: 0 auto;">
<div class="col-sm mt-3 mt-md-0">
    {% include figure.liquid loading="eager" path="assets/img/posts/2024-06-03-Controller-Design-Day-5/11_tracking_2.png" class="img-fluid rounded z-depth-1" zoomable=true %}
</div>
</div>
<div class="caption">
    Figure 11. Graphs of Joint Parameters with \( K_p = 1, K_i = 0.01, K_d = 0.1 \).
</div>

###### 3) $$K_p = 100, K_i = 1, K_d = 20$$

I further increased the gain, resulting in rapid stabilization of the multi-link mechanism. An interesting behavior observed was that the multi-link mechanism maintained an almost straight posture during its movement. Video 5 and Figure 12 display the results of the simulation.

<div style="width: 80%; margin: 0 auto; display: flex; justify-content: center;">
    <div>
        {% include video.liquid path="assets/video/posts/2024-06-03-Controller-Design-Day-5/05_tracking_3.mp4" class="img-fluid rounded z-depth-1" controls=true muted=true %}
    </div>
</div>

<div class="caption">
    Video 5. Tracking of the Triple-Link Mechanism with \( K_p = 100, K_i = 1, K_d = 20 \).
</div>

<div style="width: 80%; margin: 0 auto;">
<div class="col-sm mt-3 mt-md-0">
    {% include figure.liquid loading="eager" path="assets/img/posts/2024-06-03-Controller-Design-Day-5/12_tracking_3.png" class="img-fluid rounded z-depth-1" zoomable=true %}
</div>
</div>
<div class="caption">
    Figure 12. Graphs of Joint Parameters with \( K_p = 100, K_i = 1, K_d = 20 \).
</div>

###### 4) $$K_p = 10^6, K_i = 10^4, K_d = 2\times10^5$$

An interesting discovery was that although I could increase the PID gains indefinitely, the system's performance has a distinct upper limit. Despite higher PID gains, performance actually deteriorated. It appears that for this controller, iterative testing is necessary to determine the optimal set of gains. Video 6 and Figure 12 compare the PID settings of $$ (K_p, K_i, K_d) = (100, 1, 20) and (10^6, 10^4, 2 \times 10^5) $$.

<div style="width: 80%; margin: 0 auto; display: flex; justify-content: center;">
    <div>
        {% include video.liquid path="assets/video/posts/2024-06-03-Controller-Design-Day-5/06_tracking_4.mp4" class="img-fluid rounded z-depth-1" controls=true muted=true %}
    </div>
</div>

<div class="caption">
    Video 6. Comparison of performance with PID gains of \( (100, 1, 20) \) versus \( (10^6, 10^4, 2\times10^5) \).
</div>

<div style="width: 80%; margin: 0 auto;">
<img-comparison-slider>
  {% include figure.liquid path="assets/img/posts/2024-06-03-Controller-Design-Day-5/12_tracking_3.png" class="img-fluid rounded z-depth-1" slot="first" %}
  {% include figure.liquid path="assets/img/posts/2024-06-03-Controller-Design-Day-5/13_tracking_4.png" class="img-fluid rounded z-depth-1" slot="second" %}
</img-comparison-slider>
</div>
<div class="caption">
    Figure 13. Interactive graphs comparing PID gains of \( (100, 1, 20) \) to \( (10^6, 10^4, 2 \times 10^5) \).
</div>

###### 5) $$K_p = 10, K_i = 300, K_d = 20$$ (Unstable System)

In the final phase of my tracking experiment, I simulated an unstable system characterized by the condition $$ K_dK_p < K_i $$. As demonstrated in Video 7, Link 1, which experienced the most significant initial perturbation, exhibited signs of instability first. This instability progressively influenced Link 2 and Link 3, causing their perturbations to intensify. Ultimately, all links displayed unstable behavior. Figure 14 vividly illustrates this phenomenon, showing the angular positions of Link 2 and Link 3 diverging over time.

<div style="width: 80%; margin: 0 auto; display: flex; justify-content: center;">
    <div>
        {% include video.liquid path="assets/video/posts/2024-06-03-Controller-Design-Day-5/07_tracking_5.mp4" class="img-fluid rounded z-depth-1" controls=true muted=true %}
    </div>
</div>

<div class="caption">
    Video 7. Tracking of the Unstable Triple-Link Mechanism with \( K_p = 10, K_i = 300, K_d = 20 \).
</div>

<div style="width: 80%; margin: 0 auto;">
<div class="col-sm mt-3 mt-md-0">
    {% include figure.liquid loading="eager" path="assets/img/posts/2024-06-03-Controller-Design-Day-5/14_tracking_5.png" class="img-fluid rounded z-depth-1" zoomable=true %}
</div>
</div>
<div class="caption">
    Figure 14. Graphs of Joint Parameters with \( K_p = 10, K_i = 300, K_d = 20 \).
</div>

### 2. Regulation

For the controller regulation simulation, I configured the initial joint PID gains to (100, 1, 20), which yielded the best performance in the tracking simulation. Additionally, I set the PID gains for the moving base to (10, 0.01, 4).

###### 1) $$ \rho_0 = 2700kg/m^3, F = 100N $$

Compared to the PID + gravity compensation controller, there was less force propagation through the links, with most of the force being compensated by the rotation of Link 3, and the base remaining almost stationary. Video 7 and Figure 14 compare the simulations of PID + Gravity Compensation (PID + g) and PID + Computed Torque (PID + t).

<div style="width: 80%; margin: 0 auto;">
    <div class="col-sm mt-3 mt-md-0">
        {% include video.liquid path="assets/video/posts/2024-06-03-Controller-Design-Day-5/08_regulation_1.mp4" class="img-fluid rounded z-depth-1" controls=true muted=true %}
    </div>
</div>
<div class="caption">
    Video 8. Regulation Comparison Under 100N Disturbance: PID with Gravity Compensation vs. PID with Computed Torque.
</div>

<div style="width: 80%; margin: 0 auto;">
<img-comparison-slider>
  {% include figure.liquid path="assets/img/posts/2024-06-03-Controller-Design-Day-5/15_regulation_1_1.png" class="img-fluid rounded z-depth-1" slot="first" %}
  {% include figure.liquid path="assets/img/posts/2024-06-03-Controller-Design-Day-5/15_regulation_1_2.png" class="img-fluid rounded z-depth-1" slot="second" %}
</img-comparison-slider>
</div>
<div class="caption">
    Figure 15. Interactive Graphs of PID with Gravity Compensation vs. PID with Computed Torque.
</div>

###### 2) $$ \rho_0 = 2700kg/m^3, F = 500N $$

In Video 8, the "PID + gravity compensation" method enabled force to propagate through the links, requiring all the actuators at the joints to exert a similar effort to stabilize the system. In contrast, Video 9 demonstrates the "PID + computed torque" method, where force propagation was significantly reduced, concentrating the effort predominantly on Joint 3. This led to a longer stabilization time for the system, as the impact was not effectively absorbed by all the links and the base, but was largely managed by link 3. The differences in actuator displacement, as shown in Figure 15 and Figure 16, related to the difference in the distribution of impact absorption between the two methods.

<div class="row mt-3">
    <div class="col-sm mt-3 mt-md-0">
        {% include video.liquid path="assets/video/posts/2024-05-28-Controller-Design-Day-4/11_moving_regulation_2.mp4" class="img-fluid rounded z-depth-1" controls=true muted=true %}
    </div>
    <div class="col-sm mt-3 mt-md-0">
        {% include video.liquid path="assets/video/posts/2024-06-03-Controller-Design-Day-5/10_regulation_2.mp4" class="img-fluid rounded z-depth-1" controls=true muted=true %}
    </div>
</div>

<div class="caption">
    Video 9 - 10. Triple-Link Mechanism Simulations with PID Control: PID + gravity compensation vs. PID + computed torque.
</div>

<div style="width: 80%; margin: 0 auto;">
<swiper-container keyboard="true" navigation="true" pagination="true" pagination-clickable="true" pagination-dynamic-bullets="true" rewind="true">
  <swiper-slide>{% include figure.liquid loading="eager" path="assets/img/posts/2024-05-28-Controller-Design-Day-4/19_moving_regulation_2.png" class="img-fluid rounded z-depth-1" zoomable=true %}</swiper-slide>
  <swiper-slide>{% include figure.liquid loading="eager" path="assets/img/posts/2024-06-03-Controller-Design-Day-5/17_regulation_2.png" class="img-fluid rounded z-depth-1" zoomable=true %}</swiper-slide>
</swiper-container>
</div>
<div class="caption">
    Figure 16 - 17. Actuator Displacement Comparison in Triple-Link Mechanism Simulations: PID + gravity compensation vs. PID + computed torque.
</div>

###### 3) $$ \rho_0 = 2700kg/m^3, F = 500N, K_p = 10^6, K_i = 10^4, K_d = 2\times10^5$$

As observed during the tracking simulation, I was able to set the PID gains extremely high. With these settings, the regulation was effectively managed; the links hardly moved. Interestingly, the base moved toward the left, opposite to the direction of the external force. This is due to the efforts of the joints. The results are showcased in Video 10 and Figure 17.

<div style="width: 80%; margin: 0 auto; display: flex; justify-content: center;">
    <div>
        {% include video.liquid path="assets/video/posts/2024-06-03-Controller-Design-Day-5/11_regulation_3.mp4" class="img-fluid rounded z-depth-1" controls=true muted=true %}
    </div>
</div>

<div class="caption">
    Video 11. Regulation of the Triple-Link Mechanism with \( K_p = 10^6, K_i = 10^4, K_d = 2\times 10^5, F = 500N \).
</div>

<div style="width: 80%; margin: 0 auto;">
<div class="col-sm mt-3 mt-md-0">
    {% include figure.liquid loading="eager" path="assets/img/posts/2024-06-03-Controller-Design-Day-5/18_regulation_3.png" class="img-fluid rounded z-depth-1" zoomable=true %}
</div>
</div>
<div class="caption">
    Figure 18. Graphs of Joint Parameters with \( K_p = 10^6, K_i = 10^4, K_d = 2\times 10^5, F = 500N \).
</div>

###### 3) $$ \rho_0 = 2700kg/m^3, F = 50000N, K_p = 10^6, K_i = 10^4, K_d = 2\times10^5$$

However, as demonstrated in Video 11 and Figure 18, if the force is sufficiently strong, the base moves to the right, aligning with the direction of the external force.

<div style="width: 80%; margin: 0 auto; display: flex; justify-content: center;">
    <div>
        {% include video.liquid path="assets/video/posts/2024-06-03-Controller-Design-Day-5/12_regulation_4.mp4" class="img-fluid rounded z-depth-1" controls=true muted=true %}
    </div>
</div>

<div class="caption">
    Video 12. Regulation of the Triple-Link Mechanism with \( K_p = 10^6, K_i = 10^4, K_d = 2\times 10^5, F = 50000N \).
</div>

<div style="width: 80%; margin: 0 auto;">
<div class="col-sm mt-3 mt-md-0">
    {% include figure.liquid loading="eager" path="assets/img/posts/2024-06-03-Controller-Design-Day-5/19_regulation_4.png" class="img-fluid rounded z-depth-1" zoomable=true %}
</div>
</div>
<div class="caption">
    Figure 19. Graphs of Joint Parameters with \( K_p = 10^6, K_i = 10^4, K_d = 2\times 10^5 , F = 50000N\).
</div>

###### 4) $$ \rho_0 = 2700kg/m^3, F = 1N, K_p = 10, K_i = 300, K_d = 20 (Unstable System) $$

Similar to the tracking simulation, I also simulated the regulation of an unstable system, defined by the condition $$ K_dK_p < K_i $$. In this configuration, even a minor perturbation can cause the system to diverge rapidly. This sensitivity to disturbances highlights the challenges of controlling such a system. In our Video 12 and Figure 20, you can observe how these tiny perturbations escalated, leading to increasingly significant deviations from the stable state.

<div style="width: 80%; margin: 0 auto; display: flex; justify-content: center;">
    <div>
        {% include video.liquid path="assets/video/posts/2024-06-03-Controller-Design-Day-5/13_regulation_5.mp4" class="img-fluid rounded z-depth-1" controls=true muted=true %}
    </div>
</div>

<div class="caption">
    Video 12. Regulation of the Unstable Triple-Link Mechanism with \( K_p = 10, K_i = 30, K_d = 20, F = 1N \).
</div>

<div style="width: 80%; margin: 0 auto;">
<div class="col-sm mt-3 mt-md-0">
    {% include figure.liquid loading="eager" path="assets/img/posts/2024-06-03-Controller-Design-Day-5/20_regulation_5.png" class="img-fluid rounded z-depth-1" zoomable=true %}
</div>
</div>
<div class="caption">
    Figure 20. Graphs of Joint Parameters with \( K_p = 10, K_i = 300, K_d = 20, F = 1N \).
</div>

# IV. Moving Forward

Today, I enhanced the control of a fully actuated multi-link mechanism using a PID controller integrated with the computed torque method. Compared to standalone PID control and PID with gravity compensation, this approach demonstrated superior performance. One notable advantage is that this system achieves full linearity through feedback linearization, simplifying the control loop.

During the simulation, I observed two key findings: First, while the gains of the PID controller can be increased significantly, this does not necessarily translate to improved performance. Second, although the computed torque method markedly enhances system performance, it does not address inherent PID limitations such as the potential for overreaction. This suggests that while the PID controller remains a component, some of its fundamental drawbacks persist.

Through Days 4 and 5, I studied and implemented the feedback linearization method for nonlinear systems. Now, equipped with this understanding, I am eager to explore a variety of other nonlinear control strategies. Stay tuned for more updates as we continue to explore the fascinating world of multi-link mechanisms and tackle the challenges associated with controlling chaotic systems!

<br>
<br>
<br>

##### You can find the Simulink model on my [GitHub repository](https://github.com/geunee20/Controller_Design).
