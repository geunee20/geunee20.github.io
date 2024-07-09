---
layout: post
title: "[Controller Design] Day 4 - Fully Actuated Multi-Link Mechanism / PID Control with Gravity Compensation Method"
date: 2024-05-28 00:00:00-0400
description: Optimizing multi-link mechanism control through advanced PID strategies integrated with gravity compensation method to enhance system response and stability.
tags: Simulink Control Controller_Design
categories: Study_with_Me
disqus_comments: true
related_posts: true
thumbnail: assets/img/posts/2024-05-28-Controller-Design-Day-4/00_thumbnail.gif
images:
  compare: true
  slider: true
---

A PID control system is valued for its straightforward design, as it does not necessitate a detailed understanding of the dynamic equations governing a triple-link mechanism. However, this simplicity can also be a drawback. The system may overreact or destabilize because it does not account for complex dynamics. Additionally, PID controllers often struggle with balancing between faster response and overshoot, and designing an effective system can be challenging due to its inherent limitations in handling nonlinear behaviors.

Today, I will introduce the first nonlinear control system that combines a PID controller with gravity compensation method. This approach offers a straightforward yet effective framework for managing a triple-link mechanism, enhancing the simplicity of implementation. By integrating gravity compensation method, this system improves upon traditional PID controllers, specifically addressing the nonlinearities induced by gravitational forces and reducing the settling time.

On Day 3, I derived the equations of motion for the triple-link mechanism:

$$
\tau = M(q)\ddot{q} + h(q, \dot{q}) + g(q)
$$

Where:

- **Matrix M (Inertial Term)**: Represents the Newtonian response of the system to applied forces, accounting for the inertia of the masses.
- **Matrix h (Coriolis and Centrifugal Forces)**: Quantifies the Coriolis and centrifugal forces, dependent on system velocities and rotational dynamics.
- **Matrix g (Gravitational Term)**: Accounts for the force due to gravity acting on each mass component.

To compensate for gravity, I extract the $$g$$ matrix from the equation of motion and add it to the feedback from the PID controller. The block diagram of this system is shown in Figure 1.

<div class="col-sm mt-3 mt-md-0">
    {% include figure.liquid loading="eager" path="assets/img/posts/2024-05-28-Controller-Design-Day-4/01_block_diagram.png" class="img-fluid rounded z-depth-1" %}
</div>
<div class="caption">
    Figure 1. Block Diagram of the PID Control System with Gravity Compensation Method
</div>

# I. A Multi-Link Mechanism with a Fixed Base

### A. Overall System

The overall system is depicted in Figure 1. This system closely resembles that of Day 2, with the addition of a gravity calculation block. The angular displacement of each joint is fed into both the PID controller block and the gravity calculation block. Each block calculates its own feedback, which is then combined to determine the feedback torque needed to actuate the joints.

<div class="col-sm mt-3 mt-md-0">
    {% include figure.liquid loading="eager" path="assets/img/posts/2024-05-28-Controller-Design-Day-4/02_fixed_overall_system.png" class="img-fluid rounded z-depth-1" %}
</div>
<div class="caption">
    Figure 2. Schematic of the Multi-Link Mechanism Control System with Fixed Base
</div>

### B. Gravity Calculation Block

The gravity calculation block is relatively straightforward. Calculating the gravity term through circuit design is complex and time-consuming. Instead, I use a "MATLAB Function block," as illustrated in Figure 2. This block allows me to write a MATLAB script to compute the gravity term, which is detailed in Figure 3.

Extracting the gravity term from the equation of motion is straightforward. Given the symbolic representation of the motion equation, I apply partial differentiation with respect to $$g$$, the gravitational constant, to isolate the gravity-related terms. After this differentiation, I multiply the result by $$g$$ again to reintegrate the gravity constant into the equation. Additionally, I replace $$q_1$$ with $$q_1 + \frac{\pi}{2}$$ in the equation, as I used $$\theta$$ as the variable instead of $$q$$, in Day 3.

<div style="width: 60%; margin: 0 auto;">
<div class="col-sm mt-3 mt-md-0">
    {% include figure.liquid loading="eager" path="assets/img/posts/2024-05-28-Controller-Design-Day-4/03_gravity_block.png" class="img-fluid rounded z-depth-1" %}
</div>
</div>
<div class="caption">
    Figure 3. Gravity Calculation Block
</div>

<div class="col-sm mt-3 mt-md-0">
    {% include figure.liquid loading="eager" path="assets/img/posts/2024-05-28-Controller-Design-Day-4/04_gravity_func.png" class="img-fluid rounded z-depth-1" %}
</div>
<div class="caption">
    Figure 4. MATLAB Script for MATLAB Function Block for Gravity Calculation 
</div>

### C. Simulation

In this section, I explore the simulation, focusing on both tracking and regulation capabilities of the system with fixed base. Through these simulations, I aim to validate the effectiveness of the controllers in maintaining trajectory accuracy (tracking) and stabilizing the system against disturbances (regulation).

##### 1) Tracking

Firstly, I set all the gains of the PID controller to zero to test if compensating gravity alone could track the reference input. The results demonstrated that it could neither track the reference input nor stabilize the system. I ran the simulation for up to 10,000 seconds, but the system remained unstable. It resembled an upside-down, slow-moving triple-link mechanism, which also exhibited chaotic behavior. The simulation video and the graph for each joint parameter are displayed in Video 1 and Figure 5, respectively.

<div style="width: 80%; margin: 0 auto;">
    <div class="col-sm mt-3 mt-md-0">
        {% include video.liquid path="assets/video/posts/2024-05-28-Controller-Design-Day-4/01_fixed_no_PID.mp4" class="img-fluid rounded z-depth-1" controls=true muted=true %}
    </div>
</div>
<div class="caption">
    Video 1. Simulation of Triple-Link Mechanism with Compensating Gravity Only.
</div>

<div style="width: 80%; margin: 0 auto;">
<div class="col-sm mt-3 mt-md-0">
    {% include figure.liquid loading="eager" path="assets/img/posts/2024-05-28-Controller-Design-Day-4/05_fixed_no_PID.png" class="img-fluid rounded z-depth-1" zoomable=true %}
</div>
</div>
<div class="caption">
    Figure 5. Joint Parameter Graphs for Sole Gravity Compensation Response.
</div>

Following the initial simulation, I executed a second simulation using the same PID controller settings as on Day 2, incorporating the gravity calculation block as well. This simulation was intended to assess the impact of gravity compensation on the system's overall performance. The results, as depicted in Video 2 and Figure 6, indicate that gravity compensation slightly enhanced the tracking speed. In Video 2, the left multi-link mechanism is controlled solely by a PID controller, while the right multi-link mechanism utilizes both PID control and gravity compensation. In Figure 6, you can slide the graphs to switch between the PID (left) and PID + gravity compensation (right) data for comparison.

<div style="width: 80%; margin: 0 auto;">
    <div class="col-sm mt-3 mt-md-0">
        {% include video.liquid path="assets/video/posts/2024-05-28-Controller-Design-Day-4/02_fixed_comparison.mp4" class="img-fluid rounded z-depth-1" controls=true muted=true %}
    </div>
</div>
<div class="caption">
    Video 2. Comparison of Tracking Performance: PID Control vs. PID with Gravity Compensation.
</div>

<div style="width: 80%; margin: 0 auto;">
<img-comparison-slider>
  {% include figure.liquid path="assets/img/posts/2024-05-28-Controller-Design-Day-4/06_fixed_tracking_PID.png" class="img-fluid rounded z-depth-1" slot="first" %}
  {% include figure.liquid path="assets/img/posts/2024-05-28-Controller-Design-Day-4/06_fixed_tracking_PID+gravity.png" class="img-fluid rounded z-depth-1" slot="second" %}
</img-comparison-slider>
</div>
<div class="caption">
    Figure 6. Interactive Graphs of PID vs. PID with Gravity Compensation.
</div>

Lastly, I extracted the feedback data from each PID controller and the gravity calculation block to analyze how the gravity term influences the overall feedback. Figures 7 through 9 display the feedback from each joint.

To reduce the torque overshoot, the PID controller's gain can be adjusted by decreasing $$K_p$$ or increasing $$K_d$$, though this tends to slow the system. However, as demonstrated in Figures 7 through 9, the feedback from gravity compensation acts to counterbalance the PID controller, effectively reducing torque overshoot without significantly affecting the system's transient response time. This allows for quicker convergence.

<div style="width: 80%; margin: 0 auto;">
<swiper-container keyboard="true" navigation="true" pagination="true" pagination-clickable="true" pagination-dynamic-bullets="true" rewind="true">
  <swiper-slide>{% include figure.liquid loading="eager" path="assets/img/posts/2024-05-28-Controller-Design-Day-4/07_fixed_PID1+g1.png" class="img-fluid rounded z-depth-1" zoomable=true %}</swiper-slide>
  <swiper-slide>{% include figure.liquid loading="eager" path="assets/img/posts/2024-05-28-Controller-Design-Day-4/08_fixed_PID2+g2.png" class="img-fluid rounded z-depth-1" zoomable=true %}</swiper-slide>
  <swiper-slide>{% include figure.liquid loading="eager" path="assets/img/posts/2024-05-28-Controller-Design-Day-4/09_fixed_PID3+g3.png" class="img-fluid rounded z-depth-1" zoomable=true %}</swiper-slide>
</swiper-container>
</div>
<div class="caption">
    Figures 7 - 9. Feedback Analysis for Each Joint with PID and Gravity Compensation.
</div>

##### 2) Regulation

Through simulation of the tracking, it was found that compensating for gravity aids in faster system convergence and reduces overshoot. Additionally, systems with gravity compensation restabilized more quickly and exhibited smaller overshoots compared to those without it under similar conditions when dealing with small disturbances. However, for larger disturbances, does gravity compensation resolve issues such as overreaction and destabilization? The answer is no. Despite implementing gravity compensation, these issues remained unresolved for larger disturbances. This conclusion is supported by the results shown in Videos 3 through 5 and Figures 10 through 12.

<div style="width: 80%; margin: 0 auto;">
    <div class="col-sm mt-3 mt-md-0">
        {% include video.liquid path="assets/video/posts/2024-05-28-Controller-Design-Day-4/03_fixed_regulation_1.mp4" class="img-fluid rounded z-depth-1" controls=true muted=true %}
    </div>
</div>
<div class="caption">
    Video 3. Regulation Comparison Under 100N Disturbance: PID Control vs. PID with Gravity Compensation.
</div>

<div style="width: 80%; margin: 0 auto;">
<img-comparison-slider>
  {% include figure.liquid path="assets/img/posts/2024-05-28-Controller-Design-Day-4/10_fixed_regulation_PID_1.png" class="img-fluid rounded z-depth-1" slot="first" %}
  {% include figure.liquid path="assets/img/posts/2024-05-28-Controller-Design-Day-4/10_fixed_regulation_PID+g_1.png" class="img-fluid rounded z-depth-1" slot="second" %}
</img-comparison-slider>
</div>
<div class="caption">
    Figure 10. Interactive Graphs of PID vs. PID with Gravity Compensation.
</div>

---

<div style="width: 80%; margin: 0 auto;">
    <div class="col-sm mt-3 mt-md-0">
        {% include video.liquid path="assets/video/posts/2024-05-28-Controller-Design-Day-4/04_fixed_regulation_2.mp4" class="img-fluid rounded z-depth-1" controls=true muted=true %}
    </div>
</div>
<div class="caption">
    Video 4. Regulation Comparison Under 110N Disturbance: PID Control vs. PID with Gravity Compensation.
</div>

<div style="width: 80%; margin: 0 auto;">
<img-comparison-slider>
  {% include figure.liquid path="assets/img/posts/2024-05-28-Controller-Design-Day-4/11_fixed_regulation_PID_2.png" class="img-fluid rounded z-depth-1" slot="first" %}
  {% include figure.liquid path="assets/img/posts/2024-05-28-Controller-Design-Day-4/11_fixed_regulation_PID+g_2.png" class="img-fluid rounded z-depth-1" slot="second" %}
</img-comparison-slider>
</div>
<div class="caption">
    Figure 11. Interactive Graphs of PID vs. PID with Gravity Compensation.
</div>

---

<div style="width: 80%; margin: 0 auto;">
    <div class="col-sm mt-3 mt-md-0">
        {% include video.liquid path="assets/video/posts/2024-05-28-Controller-Design-Day-4/03_fixed_regulation_1.mp4" class="img-fluid rounded z-depth-1" controls=true muted=true %}
    </div>
</div>
<div class="caption">
    Video 5. Regulation Comparison Under 3000N Disturbance: PID Control vs. PID with Gravity Compensation.
</div>

<div style="width: 80%; margin: 0 auto;">
<img-comparison-slider>
  {% include figure.liquid path="assets/img/posts/2024-05-28-Controller-Design-Day-4/12_fixed_regulation_PID_3.png" class="img-fluid rounded z-depth-1" slot="first" %}
  {% include figure.liquid path="assets/img/posts/2024-05-28-Controller-Design-Day-4/12_fixed_regulation_PID+g_3.png" class="img-fluid rounded z-depth-1" slot="second" %}
</img-comparison-slider>
</div>
<div class="caption">
    Figure 12. Interactive Graphs of PID vs. PID with Gravity Compensation.
</div>

Even though gravity compensation could not resolve the overreaction issue, it was observed that systems with gravity compensation recovered more quickly than those without. This quicker recovery was achieved by maintaining a faster angular speed when the angular displacement was small. Through simulation, it was determined that gravity compensation is more effective when there is a small angular displacement. Consequently, under conditions of small angular displacement, using a PID controller with gravity compensation can efficiently manage the system, optimizing performance and stability.

# II. A Multi-Link Mechanism with a Moving Base

### A. Overall System

Thankfully, the base moves only along the x-axis, and there are no gravity terms related to the base. As a result, this setup closely resembles a system with a fixed base. The overall system configuration is depicted in Figure 13.

<div class="col-sm mt-3 mt-md-0">
    {% include figure.liquid loading="eager" path="assets/img/posts/2024-05-28-Controller-Design-Day-4/13_moving_overall_system.png" class="img-fluid rounded z-depth-1" zoomable=true %}
</div>
<div class="caption">
    Figure 13. Schematic of the Multi-Link Mechanism Control System with Moving Base
</div>

### B. Simulation

Since there were no significant changes, and I've already discussed all the findings in the fixed base section, I will present the results using the same parameters as Day 2. The results demonstrate faster stabilization.

##### 1) Tracking

Videos 6 through 9 and Figures 14 through 17 demonstrate the tracking capabilities of the controller across various initial states in the simulation of a triple-link mechanism with a moving base. The initial states are defined as follows: for $$q_1$$, the values are (180, -90, 150, 190); for $$q_2$$, the values are (0, 0, 180, 180); and for $$q_3$$, the values are (0, 0, 180, -110).

<div class="row mt-3">
    <div class="col-sm mt-3 mt-md-0">
        {% include video.liquid path="assets/video/posts/2024-05-28-Controller-Design-Day-4/06_moving_tracking_1.mp4" class="img-fluid rounded z-depth-1" controls=true muted=true %}
    </div>
    <div class="col-sm mt-3 mt-md-0">
        {% include video.liquid path="assets/video/posts/2024-05-28-Controller-Design-Day-4/07_moving_tracking_2.mp4" class="img-fluid rounded z-depth-1" controls=true muted=true %}
    </div>
</div>
<div class="row mt-3">
    <div class="col-sm mt-3 mt-md-0">
        {% include video.liquid path="assets/video/posts/2024-05-28-Controller-Design-Day-4/08_moving_tracking_3.mp4" class="img-fluid rounded z-depth-1" controls=true muted=true %}
    </div>
    <div class="col-sm mt-3 mt-md-0">
        {% include video.liquid path="assets/video/posts/2024-05-28-Controller-Design-Day-4/09_moving_tracking_4.mp4" class="img-fluid rounded z-depth-1" controls=true muted=true %}
    </div>
</div>
<div class="caption">
    Videos 6 - 9. Demonstrating the Tracking Capabilities of the Controller Across Various Initial States in the Triple-Link Mechanism with Moving Base Simulation.
</div>

<div style="width: 80%; margin: 0 auto;">
<swiper-container keyboard="true" navigation="true" pagination="true" pagination-clickable="true" pagination-dynamic-bullets="true" rewind="true">
  <swiper-slide>{% include figure.liquid loading="eager" path="assets/img/posts/2024-05-28-Controller-Design-Day-4/14_moving_tracking_1.png" class="img-fluid rounded z-depth-1" zoomable=true %}</swiper-slide>
  <swiper-slide>{% include figure.liquid loading="eager" path="assets/img/posts/2024-05-28-Controller-Design-Day-4/15_moving_tracking_2.png" class="img-fluid rounded z-depth-1" zoomable=true %}</swiper-slide>
  <swiper-slide>{% include figure.liquid loading="eager" path="assets/img/posts/2024-05-28-Controller-Design-Day-4/16_moving_tracking_3.png" class="img-fluid rounded z-depth-1" zoomable=true %}</swiper-slide>
  <swiper-slide>{% include figure.liquid loading="eager" path="assets/img/posts/2024-05-28-Controller-Design-Day-4/17_moving_tracking_4.png" class="img-fluid rounded z-depth-1" zoomable=true %}</swiper-slide>
</swiper-container>
</div>
<div class="caption">
    Figures 14 - 17. Analysis of Angular Displacement in the Triple-Link Mechanism with Moving Base Across Various Initial States.
</div>

##### 2) Regulation

Videos 10 through 13 and Figures 18 through 21 demonstrate the tracking capabilities of the controller across various initial states in the simulation of a triple-link mechanism with a moving base. All the initial states, $$q$$, were set to 0. Adjustments were then made to the density of the base and the magnitude of disturbance. The densities, denoted as $$\rho_0$$, are set to $$(2700, 2700, 50000, 50000) kg/m^3$$, and the forces, denoted as $$f$$, are $$(100, 500, 300, 500) N$$.

<div class="row mt-3">
    <div class="col-sm mt-3 mt-md-0">
        {% include video.liquid path="assets/video/posts/2024-05-28-Controller-Design-Day-4/10_moving_regulation_1.mp4" class="img-fluid rounded z-depth-1" controls=true muted=true %}
    </div>
    <div class="col-sm mt-3 mt-md-0">
        {% include video.liquid path="assets/video/posts/2024-05-28-Controller-Design-Day-4/11_moving_regulation_2.mp4" class="img-fluid rounded z-depth-1" controls=true muted=true %}
    </div>
</div>
<div class="row mt-3">
    <div class="col-sm mt-3 mt-md-0">
        {% include video.liquid path="assets/video/posts/2024-05-28-Controller-Design-Day-4/12_moving_regulation_3.mp4" class="img-fluid rounded z-depth-1" controls=true muted=true %}
    </div>
    <div class="col-sm mt-3 mt-md-0">
        {% include video.liquid path="assets/video/posts/2024-05-28-Controller-Design-Day-4/13_moving_regulation_4.mp4" class="img-fluid rounded z-depth-1" controls=true muted=true %}
    </div>
</div>
<div class="caption">
    Videos 10 - 13. Demonstrating the Regulation Capabilities of the Controller Across Various Disturbances in the Triple-Link Mechanism with Moving Base Simulation.
</div>

<div style="width: 80%; margin: 0 auto;">
<swiper-container keyboard="true" navigation="true" pagination="true" pagination-clickable="true" pagination-dynamic-bullets="true" rewind="true">
  <swiper-slide>{% include figure.liquid loading="eager" path="assets/img/posts/2024-05-28-Controller-Design-Day-4/18_moving_regulation_1.png" class="img-fluid rounded z-depth-1" zoomable=true %}</swiper-slide>
  <swiper-slide>{% include figure.liquid loading="eager" path="assets/img/posts/2024-05-28-Controller-Design-Day-4/19_moving_regulation_2.png" class="img-fluid rounded z-depth-1" zoomable=true %}</swiper-slide>
  <swiper-slide>{% include figure.liquid loading="eager" path="assets/img/posts/2024-05-28-Controller-Design-Day-4/20_moving_regulation_3.png" class="img-fluid rounded z-depth-1" zoomable=true %}</swiper-slide>
  <swiper-slide>{% include figure.liquid loading="eager" path="assets/img/posts/2024-05-28-Controller-Design-Day-4/21_moving_regulation_4.png" class="img-fluid rounded z-depth-1" zoomable=true %}</swiper-slide>
</swiper-container>
</div>
<div class="caption">
    Figures 18 - 21. Analysis of Angular Displacement in the Triple-Link Mechanism with Moving Base Across Various Disturbances.
</div>

# III. Moving Forward

Today, I managed a fully actuated multi-link mechanism using a PID controller complemented by gravity compensation. The simulations revealed that gravity compensation significantly enhances stabilization by maintaining a high angular velocity when the angular displacement is minimal. Upon examining the torque feedback, I observed that gravity compensation acts as a counterbalance to the feedback from the PID, effectively reducing torque overshoot. Therefore, in scenarios involving only slight angular displacements, incorporating gravity compensation can markedly improve system performance.

During today's simulations, I realized that simulating the fixed base system might be time-consuming. Hence, starting from Day 5, I will omit simulations of the fixed base system. Stay tuned for more updates as we continue exploring the fascinating realm of multi-link mechanisms and tackle the challenges associated with controlling chaotic systems!

<br>
<br>
<br>

##### You can find the Simulink model on my [GitHub repository](https://github.com/geunee20/Controller_Design).
