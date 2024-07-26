---
layout: post
title: "[Controller Design] Day 2: PID Controller"
date: 2024-05-20 00:00:00-0400
description: An in-depth exploration of a fully actuated triple-link mechanism using a PID controller, with its tracking accuracy, and regulation effectiveness.
tags: Simulink PID
categories: Study_with_Me Controller_Design
disqus_comments: true
related_posts: true
thumbnail: assets/img/posts/2024-05-20-Controller-Design-Day-2/00_thumbnail.gif
images:
  compare: true
  slider: true
---

A fully actuated system is one in which the number of independent actuators matches the number of degrees of freedom, allowing for direct control over each motion dimension. This setup is typical in industrial robots and other motor-driven mechanisms, where each joint or axis is controlled by its own motor, facilitating precise manipulation.

For example, a fully actuated multi-link mechanism is easier to control because each degree of freedom—whether it involves rotation or translation—is directly managed by an independent actuator. This direct manipulation allows for precise adjustments to the multi-link mechanism’s position and motion, simplifying the control algorithms needed for maintaining stability and achieving desired behaviors. In such systems, the control can often be accomplished using linear controllers, though an understanding of nonlinear dynamics is still beneficial for optimizing performance and handling more complex scenarios.

Today, I will simulate the PID controller, which is the simplest and most representative type of linear controller. The block diagram of the PID controller system is presented in Figure 0. I added this at the end and chose not to update the figure numbering.

<div class="col-sm mt-3 mt-md-0">
    {% include figure.liquid loading="eager" path="assets/img/posts/2024-05-20-Controller-Design-Day-2/00_block_diagram.png" class="img-fluid rounded z-depth-1" %}
</div>
<div class="caption">
    Figure 0. Block Diagram of the PID Controller System
</div>

# I. A Multi-Link Mechanism with a Fixed Base

### A. Overall System

The overall system is depicted in Figure 1. This system closely resembles that of Day 1 but includes an added PID control subsystem. The angular displacement of each joint is input into the PID controller block, which calculates the feedback torque to actuate the joints.

<div class="col-sm mt-3 mt-md-0">
    {% include figure.liquid loading="eager" path="assets/img/posts/2024-05-20-Controller-Design-Day-2/01_fixed_overall_system.png" class="img-fluid rounded z-depth-1" %}
</div>
<div class="caption">
    Figure 1. Schematic of the Multi-Link Mechanism Control System with Fixed Base
</div>

### B. Subsystems

##### 1) Links

In the updated design of the link subsystems, illustrated in Figures 2, 3, and 4, I've introduced several modifications to enhance realism and functionality. These changes echo the foundational setups from Day 1 but with key differences:

- To better realize the oscillatory nature of real-world multi-link mechanism dynamics, I've configured the angular displacement of each link to be periodic, spanning a full circle from $$ -\pi $$ to $$ \pi $$.
- In a shift from the initial configuration, I've realigned the x-axis of link 1 to coincide with the z-axis of the world frame.
- Additionally, I've incorporated blocks to generate an impulsive external force at the end of link 3, to simulate external forces on the system and test its regulatory response.

The modifications ensure that when the multi-link mechanism is upright, its angular displacement is set to $$ 0 $$. This allows us to use the origin as our equilibrium point in subsequent simulations.

<div class="col-sm mt-3 mt-md-0">
    <swiper-container keyboard="true" navigation="true" pagination="true" pagination-clickable="true" pagination-dynamic-bullets="true" rewind="true">
        <swiper-slide>
            {% include figure.liquid loading="eager" path="assets/img/posts/2024-05-20-Controller-Design-Day-2/02_subsystem_link1.png" class="img-fluid rounded z-depth-1" %}
            <div class="caption">
                Figure 2: Detailed View of Link 1 Subsystem.
            </div>
        </swiper-slide>
        <swiper-slide>
            {% include figure.liquid loading="eager" path="assets/img/posts/2024-05-20-Controller-Design-Day-2/03_subsystem_link2.png" class="img-fluid rounded z-depth-1" %}
            <div class="caption">
                Figure 3: Detailed View of Link 2 Subsystem.
            </div>
        </swiper-slide>
        <swiper-slide>
            {% include figure.liquid loading="eager" path="assets/img/posts/2024-05-20-Controller-Design-Day-2/04_subsystem_link3.png" class="img-fluid rounded z-depth-1" %}
            <div class="caption">
                Figure 4: Detailed View of Link 3 Subsystem.
            </div>
        </swiper-slide>
    </swiper-container>
</div>

##### 2) PID Controller

Figure 5 shows a PID controller subsystem. It begins by comparing a desired target (q_d) with an actual value (q) to determine the error, or the difference between what is desired and what exists. This error is then processed through three types of control: Proportional (Kp), which reacts to the current error; Integral (Ki), which addresses the accumulation of past errors; and Derivative (Kd), which predicts future errors. The outputs from these controls are combined to produce a feedback signal that adjusts the system to minimize the error, helping maintain the desired output.

<div class="col-sm mt-3 mt-md-0">
    {% include figure.liquid loading="eager" path="assets/img/posts/2024-05-20-Controller-Design-Day-2/05_subsystem_PID_controller.png" class="img-fluid rounded z-depth-1" %}
</div>
<div class="caption">
    Figure 5. Detailed View of PID Controller Subsystem.
</div>

### C. Simulation

In this section, I explore the simulation, focusing on both tracking and regulation capabilities of the system with fixed base. Through these simulations, I aim to validate the effectiveness of the controllers in maintaining trajectory accuracy (tracking) and stabilizing the system against disturbances (regulation).

##### 1) Tracking

Videos 1 through 4 demonstrate the behavior of the system from various initial states. Figures 6 through 9 present an analysis of angular displacement for different initial conditions. These results show the robust tracking capabilities of the control system, which efficiently elevates the multi-link mechanism to a vertical position from any initial state.

<div class="row mt-3">
    <div class="col-sm mt-3 mt-md-0">
        {% include video.liquid path="assets/video/posts/2024-05-20-Controller-Design-Day-2/01_fixed_tracking_1.mp4" class="img-fluid rounded z-depth-1" controls=true muted=true %}
    </div>
    <div class="col-sm mt-3 mt-md-0">
        {% include video.liquid path="assets/video/posts/2024-05-20-Controller-Design-Day-2/02_fixed_tracking_2.mp4" class="img-fluid rounded z-depth-1" controls=true muted=true %}
    </div>
</div>
<div class="row mt-3">
    <div class="col-sm mt-3 mt-md-0">
        {% include video.liquid path="assets/video/posts/2024-05-20-Controller-Design-Day-2/03_fixed_tracking_3.mp4" class="img-fluid rounded z-depth-1" controls=true muted=true %}
    </div>
    <div class="col-sm mt-3 mt-md-0">
        {% include video.liquid path="assets/video/posts/2024-05-20-Controller-Design-Day-2/04_fixed_tracking_4.mp4" class="img-fluid rounded z-depth-1" controls=true muted=true %}
    </div>
</div>
<div class="caption">
    Videos 1 - 4. Demonstrating the Tracking Capabilities of the Controller Across Various Initial States in the Triple-Link Mechanism with Fixed Base Simulation.
</div>

<div style="width: 80%; margin: 0 auto;">
<swiper-container keyboard="true" navigation="true" pagination="true" pagination-clickable="true" pagination-dynamic-bullets="true" rewind="true">
  <swiper-slide>{% include figure.liquid loading="eager" path="assets/img/posts/2024-05-20-Controller-Design-Day-2/06_fixed_tracking_1.png" class="img-fluid rounded z-depth-1" zoomable=true %}</swiper-slide>
  <swiper-slide>{% include figure.liquid loading="eager" path="assets/img/posts/2024-05-20-Controller-Design-Day-2/07_fixed_tracking_2.png" class="img-fluid rounded z-depth-1" zoomable=true %}</swiper-slide>
  <swiper-slide>{% include figure.liquid loading="eager" path="assets/img/posts/2024-05-20-Controller-Design-Day-2/08_fixed_tracking_3.png" class="img-fluid rounded z-depth-1" zoomable=true %}</swiper-slide>
  <swiper-slide>{% include figure.liquid loading="eager" path="assets/img/posts/2024-05-20-Controller-Design-Day-2/09_fixed_tracking_4.png" class="img-fluid rounded z-depth-1" zoomable=true %}</swiper-slide>
</swiper-container>
</div>
<div class="caption">
    Figures 6 - 9. Analysis of Angular Displacement in the Triple-Link Mechanism with Fixed Base Across Various Initial States.
</div>

##### 2) Regulation

Videos 5 through 9 demonstrate the robust regulating capabilities of the control system, which efficiently rejects disturbances and maintains the reference position. Meanwhile, Figures 6 through 9 analyze the angular displacement under various disturbances. Specifically, Videos 6 and 7, along with Figures 11 and 12, showcase a simulation of a triple-link mechanism subjected to a substantial disturbance, significant enough to potentially cause a single rotation. However, in these demonstrations, the multi-link mechanism rotates almost twice. Similarly, As demonstrated in Video 8 and Figure 13, the system exhibits prolonged regulation times when subjected to a strong disturbance. I suspect this excessive rotation is due to the control system not accounting for the angular velocity and angular acceleration of the multi-link mechanism. This oversight in the control strategy leads to an over-response to the disturbance. I am uncertain whether this issue can be addressed by tuning the PID controller or changing system parameters. Please leave a comment if you have any ideas.

<div class="row mt-3">
    <div class="col-sm mt-3 mt-md-0">
        {% include video.liquid path="assets/video/posts/2024-05-20-Controller-Design-Day-2/05_fixed_regulation_1.mp4" class="img-fluid rounded z-depth-1" controls=true muted=true %}
    </div>
    <div class="col-sm mt-3 mt-md-0">
        {% include video.liquid path="assets/video/posts/2024-05-20-Controller-Design-Day-2/06_fixed_regulation_2.mp4" class="img-fluid rounded z-depth-1" controls=true muted=true %}
    </div>
</div>
<div class="row mt-3">
    <div class="col-sm mt-3 mt-md-0">
        {% include video.liquid path="assets/video/posts/2024-05-20-Controller-Design-Day-2/07_fixed_regulation_3.mp4" class="img-fluid rounded z-depth-1" controls=true muted=true %}
    </div>
    <div class="col-sm mt-3 mt-md-0">
        {% include video.liquid path="assets/video/posts/2024-05-20-Controller-Design-Day-2/08_fixed_regulation_4.mp4" class="img-fluid rounded z-depth-1" controls=true muted=true %}
    </div>
</div>
<div class="caption">
    Videos 5 - 8. Demonstrating the Regulation Capabilities of the Controller Across Various Disturbances in the Triple-Link Mechanism with Fixed Base Simulation.
</div>

<div style="width: 80%; margin: 0 auto;">
<swiper-container keyboard="true" navigation="true" pagination="true" pagination-clickable="true" pagination-dynamic-bullets="true" rewind="true">
  <swiper-slide>{% include figure.liquid loading="eager" path="assets/img/posts/2024-05-20-Controller-Design-Day-2/10_fixed_regulation_1.png" class="img-fluid rounded z-depth-1" zoomable=true %}</swiper-slide>
  <swiper-slide>{% include figure.liquid loading="eager" path="assets/img/posts/2024-05-20-Controller-Design-Day-2/11_fixed_regulation_2.png" class="img-fluid rounded z-depth-1" zoomable=true %}</swiper-slide>
  <swiper-slide>{% include figure.liquid loading="eager" path="assets/img/posts/2024-05-20-Controller-Design-Day-2/12_fixed_regulation_3.png" class="img-fluid rounded z-depth-1" zoomable=true %}</swiper-slide>
  <swiper-slide>{% include figure.liquid loading="eager" path="assets/img/posts/2024-05-20-Controller-Design-Day-2/13_fixed_regulation_4.png" class="img-fluid rounded z-depth-1" zoomable=true %}</swiper-slide>
</swiper-container>
</div>
<div class="caption">
    Figures 10 - 13. Analysis of Angular Displacement in the Triple-Link Mechanism with Fixed Base Across Various Disturbances.
</div>

# II. A Multi-Link Mechanism with a Moving Base

### A. Overall System

The overall system configuration of the triple-link mechanism with a moving base is similar to that of the fixed base, with the addition of a moving frame. Also, I have increased the density of the base to make it heavier.

<div class="col-sm mt-3 mt-md-0">
    {% include figure.liquid loading="eager" path="assets/img/posts/2024-05-20-Controller-Design-Day-2/14_moving_overall_system.png" class="img-fluid rounded z-depth-1" %}
</div>
<div class="caption">
    Figure 14. Schematic of the Multi-Link Mechanism Control System with Moving Base
</div>

### B. Simulation

In this section, I explore the simulation, focusing on both tracking and regulation capabilities of the system with moving base. Through these simulations, I aim to validate the effectiveness of the controllers in maintaining trajectory accuracy (tracking) and stabilizing the system against disturbances (regulation).

##### 1) Tracking

Videos 9 through 12 showcase the robust tracking capabilities of the control system, which efficiently brings the multi-link mechanism to a vertical position from any starting point. Meanwhile, Figures 15 through 18 provide an analysis of angular displacement across various initial states. Interestingly, the trajectory of the system with a moving base appears more sensible than that of the system with a fixed base, likely due to the addition of an extra degree of freedom.

<div class="row mt-3">
    <div class="col-sm mt-3 mt-md-0">
        {% include video.liquid path="assets/video/posts/2024-05-20-Controller-Design-Day-2/09_moving_tracking_1.mp4" class="img-fluid rounded z-depth-1" controls=true muted=true %}
    </div>
    <div class="col-sm mt-3 mt-md-0">
        {% include video.liquid path="assets/video/posts/2024-05-20-Controller-Design-Day-2/10_moving_tracking_2.mp4" class="img-fluid rounded z-depth-1" controls=true muted=true %}
    </div>
</div>
<div class="row mt-3">
    <div class="col-sm mt-3 mt-md-0">
        {% include video.liquid path="assets/video/posts/2024-05-20-Controller-Design-Day-2/11_moving_tracking_3.mp4" class="img-fluid rounded z-depth-1" controls=true muted=true %}
    </div>
    <div class="col-sm mt-3 mt-md-0">
        {% include video.liquid path="assets/video/posts/2024-05-20-Controller-Design-Day-2/12_moving_tracking_4.mp4" class="img-fluid rounded z-depth-1" controls=true muted=true %}
    </div>
</div>
<div class="caption">
    Videos 9 - 12. Demonstrating the Tracking Capabilities of the Controller Across Various Initial States in the Triple-Link Mechanism with Moving Base Simulation.
</div>

<div style="width: 80%; margin: 0 auto;">
<swiper-container keyboard="true" navigation="true" pagination="true" pagination-clickable="true" pagination-dynamic-bullets="true" rewind="true">
  <swiper-slide>{% include figure.liquid loading="eager" path="assets/img/posts/2024-05-20-Controller-Design-Day-2/15_moving_tracking_1.png" class="img-fluid rounded z-depth-1" zoomable=true %}</swiper-slide>
  <swiper-slide>{% include figure.liquid loading="eager" path="assets/img/posts/2024-05-20-Controller-Design-Day-2/16_moving_tracking_2.png" class="img-fluid rounded z-depth-1" zoomable=true %}</swiper-slide>
  <swiper-slide>{% include figure.liquid loading="eager" path="assets/img/posts/2024-05-20-Controller-Design-Day-2/17_moving_tracking_3.png" class="img-fluid rounded z-depth-1" zoomable=true %}</swiper-slide>
  <swiper-slide>{% include figure.liquid loading="eager" path="assets/img/posts/2024-05-20-Controller-Design-Day-2/18_moving_tracking_4.png" class="img-fluid rounded z-depth-1" zoomable=true %}</swiper-slide>
</swiper-container>
</div>
<div class="caption">
    Figures 15 - 18. Analysis of Angular Displacement in the Triple-Link Mechanism with Moving Base Across Various Initial States.
</div>

##### 2) Regulation

Videos 13 through 16 illustrate the regulating capabilities of the control system, which effectively rejects disturbances and maintains the reference position in most scenarios. Meanwhile, Figures 19 through 22 provide an analysis of angular displacement under various disturbances.

Specifically, Videos 13 to 15 demonstrate conditions with low base density under mild and strong disturbances, and very high base density under strong disturbance, respectively. Under these setups, the system regulates effectively, either by the base dispersing the energy or functioning as a fixed base. However, as shown in Video 16, under certain combinations of base density and disturbance intensity, the regulation is not successful, and the system becomes unstable. This instability is likely because, similar to the fixed base scenario, the PID controller primarily considers angular displacement, which may not suffice under more challenging conditions.

<div class="row mt-3">
    <div class="col-sm mt-3 mt-md-0">
        {% include video.liquid path="assets/video/posts/2024-05-20-Controller-Design-Day-2/13_moving_regulation_1.mp4" class="img-fluid rounded z-depth-1" controls=true muted=true %}
    </div>
    <div class="col-sm mt-3 mt-md-0">
        {% include video.liquid path="assets/video/posts/2024-05-20-Controller-Design-Day-2/14_moving_regulation_2.mp4" class="img-fluid rounded z-depth-1" controls=true muted=true %}
    </div>
</div>
<div class="row mt-3">
    <div class="col-sm mt-3 mt-md-0">
        {% include video.liquid path="assets/video/posts/2024-05-20-Controller-Design-Day-2/15_moving_regulation_3.mp4" class="img-fluid rounded z-depth-1" controls=true muted=true %}
    </div>
    <div class="col-sm mt-3 mt-md-0">
        {% include video.liquid path="assets/video/posts/2024-05-20-Controller-Design-Day-2/16_moving_regulation_4.mp4" class="img-fluid rounded z-depth-1" controls=true muted=true %}
    </div>
</div>
<div class="caption">
    Videos 13 - 16. Demonstrating the Regulation Capabilities of the Controller Across Various Disturbances in the Triple-Link Mechanism with Moving Base Simulation.
</div>

<div style="width: 80%; margin: 0 auto;">
<swiper-container keyboard="true" navigation="true" pagination="true" pagination-clickable="true" pagination-dynamic-bullets="true" rewind="true">
  <swiper-slide>{% include figure.liquid loading="eager" path="assets/img/posts/2024-05-20-Controller-Design-Day-2/19_moving_regulation_1.png" class="img-fluid rounded z-depth-1" zoomable=true %}</swiper-slide>
  <swiper-slide>{% include figure.liquid loading="eager" path="assets/img/posts/2024-05-20-Controller-Design-Day-2/20_moving_regulation_2.png" class="img-fluid rounded z-depth-1" zoomable=true %}</swiper-slide>
  <swiper-slide>{% include figure.liquid loading="eager" path="assets/img/posts/2024-05-20-Controller-Design-Day-2/21_moving_regulation_3.png" class="img-fluid rounded z-depth-1" zoomable=true %}</swiper-slide>
  <swiper-slide>{% include figure.liquid loading="eager" path="assets/img/posts/2024-05-20-Controller-Design-Day-2/22_moving_regulation_4.png" class="img-fluid rounded z-depth-1" zoomable=true %}</swiper-slide>
</swiper-container>
</div>
<div class="caption">
    Figures 19 - 22. Analysis of Angular Displacement in the Triple-Link Mechanism with Moving Base Across Various Disturbances.
</div>

# III. Moving Forward

Today, I dealt with a fully actuated multi-link mechanism using a PID controller. This control system offers the benefit of a relatively simple design, as it does not require consideration of the complete dynamic equations of the triple-link mechanism. However, this simplicity sometimes leads to drawbacks, such as overreaction or subsequent destabilization of the system. In my next post, I will explore alternative control systems that can address these issues. Stay tuned for more updates as we delve deeper into the intriguing world of multi-link mechanisms and confront the challenges of controlling chaotic systems!

<br>
<br>
<br>

##### You can find the Simulink model on my [GitHub repository](https://github.com/geunee20/Controller_Design).
