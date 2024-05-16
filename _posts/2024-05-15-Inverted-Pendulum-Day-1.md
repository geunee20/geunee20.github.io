---
layout: post
title: "[Inverted Pendulum] Day 1 - Creating the Multibody System Using Simscape"
date: 2024-05-15 11:31:00-0400
description: Exploring the dynamics of a pendulum by constructing and simulating a multibody system using Simscape.
tags: Simulink Control Inverted_Pendulum
categories: Study_with_Me
disqus_comments: true
related_posts: true
---

Today marked the exciting beginning of my project on simulating inverted pendulum control, a pivotal concept in dynamic systems and control theory. My objective for Day 1 was to construct the foundational multibody system using Simscape, which involved creating pendulums with varying numbers of links. The challenge here is not just about assembling a physical model but simulating how these systems behave under different conditions, a crucial aspect for anyone venturing into the world of advanced robotics and control systems.

### Overview of the Pendulum System

The process began with setting up the basic structure of the pendulum system. I designed a schematic that includes various components like the pendulum itself, connectors, and the control systems that manage the pendulum's movement (See Figure 1). This visual setup helps in understanding how each part interacts within the system and is a good starting point for simulation.

<div class="col-sm mt-3 mt-md-0">
    {% include figure.liquid loading="eager" path="assets/img/posts/2024-05-15-Inverted-Pendulum-Day-1/1_overall_system.png" class="img-fluid rounded z-depth-1" %}
</div>
<div class="caption">
    Figure 1. Schematic of the Pendulum System Setup
</div>

### Subsystem Detailing

Next, I zoomed into the subsystems, focusing on the connectors that play a critical role in the pendulum's dynamics. These connectors determine how the pendulum reacts to forces and movements applied to it, which is essential for analyzing stability and control (See Figure 2). Understanding each component is vital for refining the simulation and ensuring accurate results.

In Figure 2, the subsystem is depicted with several connectors, each highlighted in different colors to indicate their specific functions:

- **Red Boxes**: The connectors within the red boxes are crucial for changing the rotational axis of the joints. Since the default rotational axis in our simulation is the z-axis, these connectors are adjusted to change this to the y-axis.

- **Blue Boxes**: The connectors within the blue boxes are used for linking the joints and the pendulum links together.

- **Green Box**: The connector in the green box is positioned at the end of the link. This setup is typically used for attaching additional components or for applying external forces to the system.

By meticulously arranging these connectors and understanding their specific roles, I can create a versatile simulation environment that accurately mimics real-world physical conditions. This level of detail is imperative for developing effective control strategies and for the thorough analysis of the pendulum dynamics.

<div class="col-sm mt-3 mt-md-0">
    {% include figure.liquid loading="eager" path="assets/img/posts/2024-05-15-Inverted-Pendulum-Day-1/2_sub_system.png" class="img-fluid rounded z-depth-1" %}
</div>
<div class="caption">
    Figure 2: Detailed View of Subsystem Connectors
</div>

### Parameter Customization

I utilized the mask editor in Simscape to enhance subsystem customization. This tool allowed me to encapsulate subsystems, thereby facilitating easy adjustments of parameters such as mass, length, and friction through a user-friendly interface. By selecting the “Edit” button, which is highlighted in a red box in Figure 3, I could seamlessly add new parameters to the subsystems.

<div class="col-sm mt-3 mt-md-0">
    {% include figure.liquid loading="eager" path="assets/img/posts/2024-05-15-Inverted-Pendulum-Day-1/3_parameter_customization.png" class="img-fluid rounded z-depth-1" %}
</div>
<div class="caption">
    Figure 3: Mask Editor Interface in Simscape
</div>

Figure 4 displays the interface after these additions. This capability is crucial for testing varied scenarios efficiently by modifying parameters without altering the core model structure. It’s particularly fascinating to observe how slight variations in a single parameter can significantly influence the system’s dynamics, potentially mimicking complex real-world behaviors.

<div style="width: 60%; margin: 0 auto;">
    <div class="col-sm mt-3 mt-md-0">
        {% include figure.liquid loading="eager" path="assets/img/posts/2024-05-15-Inverted-Pendulum-Day-1/4_parameter_setting.png" class="img-fluid rounded z-depth-1" %}
    </div>
</div>
<div class="caption">
    Figure 4: Parameter Interface in Simscape After Masking
</div>

### Simulation Results

I am excited to showcase initial results from running simulations with our system under varying initial conditions, providing deep insights into its dynamics.

Figure 5 illustrates a non-chaotic motion resulting from a specific initial angle and pendulum length. This setup reveals predictable and stable behavior, demonstrating how controlled conditions can ensure system stability.

<div style="width: 60%; margin: 0 auto;">
    <div class="col-sm mt-3 mt-md-0">
        {% include figure.liquid loading="eager" path="assets/img/posts/2024-05-15-Inverted-Pendulum-Day-1/5_non_chaotic_motion.gif" class="img-fluid rounded z-depth-1" %}
    </div>
</div>
<div class="caption">
    Figure 5: Demonstration of Non-Chaotic Motion in the Triple Pendulum Simulation
</div>

Figure 6 captures the onset of chaotic behavior under a slightly altered set of conditions. A small change in the initial angle leads to unpredictable and complex dynamics. This showcases the system's sensitive dependence on initial conditions—a defining characteristic of chaotic systems.

<div style="width: 60%; margin: 0 auto;">
    <div class="col-sm mt-3 mt-md-0">
        {% include figure.liquid loading="eager" path="assets/img/posts/2024-05-15-Inverted-Pendulum-Day-1/6_chaotic_motion.gif" class="img-fluid rounded z-depth-1" %}
    </div>
</div>
<div class="caption">
    Figure 6: Demonstration of Chaotic Motion in the Triple Pendulum Simulation
</div>

Figure 7 provides a detailed analysis of the angular displacement for each joint of the triple pendulum system under non-chaotic conditions. This illustrates how each joint behaves in terms of movement and speed when the system is stable and predictable.

<div style="width: 60%; margin: 0 auto;">
    <div class="col-sm mt-3 mt-md-0">
        {% include figure.liquid loading="eager" path="assets/img/posts/2024-05-15-Inverted-Pendulum-Day-1/7_non_chaotic.png" class="img-fluid rounded z-depth-1" %}
    </div>
</div>
<div class="caption">
    Figure 7: Angular Displacement Analysis of the Triple Pendulum System 
</div>

Conversely, Figure 8 showcases the same parameter under chaotic conditions, highlighting the significant differences when small changes in initial conditions lead to complex and unpredictable dynamics. Comparing these images helps underline the critical impact of initial setup variations on the system’s behavior, offering valuable insights for designing mechanisms to control or utilize chaotic responses effectively.

<div style="width: 60%; margin: 0 auto;">
    <div class="col-sm mt-3 mt-md-0">
        {% include figure.liquid loading="eager" path="assets/img/posts/2024-05-15-Inverted-Pendulum-Day-1/8_chaotic.png" class="img-fluid rounded z-depth-1" %}
    </div>
</div>
<div class="caption">
    Figure 8: Angular Displacement Analysis of the Triple Pendulum System 
</div>

### Moving Forward

I chose different lengths for the pendulum primarily due to controllability concerns. Varying the lengths has helped in understanding how each variation impacts the system's ability to be controlled, thus informing the development of more effective control strategies.

In the next post, I plan to delve deeper into the relationship between the length of each link and its [controllability](https://en.wikipedia.org/wiki/Controllability). This analysis is crucial as it will inform the implementation of a PID controller for the pendulum with a fixed base. Understanding these relationships is key to developing effective control strategies that can manage the complexity and inherent challenges of chaotic systems.

It's a blend of theory and practical application, and I'm eager to see where this journey takes me in the realm of control engineering. Stay tuned for more updates as I explore the fascinating world of inverted pendulums and tackle the challenges of controlling chaotic systems!

<br>
<br>
<br>

##### You can find the Simulink model on my [GitHub repository](https://github.com/geunee20/Inverted_Pendulum).
