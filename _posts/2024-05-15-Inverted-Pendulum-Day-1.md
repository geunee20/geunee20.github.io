---
layout: post
title: "[Inverted Pendulum] Day 1 - Creating the Multibody System Using Simscape"
date: 2024-05-15 00:00:00-0000
description: Exploring the dynamics of a pendulum by constructing and simulating a multibody system using Simscape.
tags: Simulink Control Inverted_Pendulum
categories: Study_with_Me
disqus_comments: true
related_posts: true
thumbnail: assets/img/posts/2024-05-15-Inverted-Pendulum-Day-1/00_thumbnail.gif
---

Today marked the exciting beginning of my project on simulating inverted pendulum control, a pivotal concept in dynamic systems and control theory. My objective for Day 1 was to construct the foundational multibody system using Simscape, which involved creating pendulums with varying numbers of links. The challenge here is not just about assembling a physical model, but also simulating how these systems behave under different conditions—a crucial aspect for anyone venturing into the world of advanced robotics and control systems.

# I. A Pendulum System with a Fixed Base

### A. Overall System

The process began with setting up the basic structure of the pendulum system with a fixed base. I designed a schematic that includes components like the links and the fixed base that manage the pendulum's movement (See Figure 1). This visual setup helps in understanding how each part interacts within the system and serves as a good starting point for simulation.

<div class="col-sm mt-3 mt-md-0">
    {% include figure.liquid loading="eager" path="assets/img/posts/2024-05-15-Inverted-Pendulum-Day-1/01_overall_system_fixed.png" class="img-fluid rounded z-depth-1" %}
</div>
<div class="caption">
    Figure 1. Schematic of the Pendulum System with Fixed Base.
</div>

### B. Subsystem - Link

Next, I focused on the Links, zooming into the subsystem that plays a critical role in the pendulum's dynamics. The revolute joint determine how the pendulum reacts to forces and movements applied to it, which is essential for analyzing stability and control (See Figure 2). Understanding each component is vital for refining the simulation and ensuring accurate results.

In Figure 2, the subsystem of link 1 is depicted with several blocks.

<div class="col-sm mt-3 mt-md-0">
    {% include figure.liquid loading="eager" path="assets/img/posts/2024-05-15-Inverted-Pendulum-Day-1/02_sub_system_link1.png" class="img-fluid rounded z-depth-1" %}
</div>
<div class="caption">
    Figure 2. Detailed View of Link 1 Subsystem.
</div>

Each block is enclosed in a color-coded box to indicate its specific function:

- **Red Boxes**: Blocks within the red boxes are crucial for adjusting the joint's frame so that the joint's rotational axis (default z-axis) aligns with the world frame's y-axis.

- **Blue Boxes**: Blocks within the blue boxes are used for linking the joints and the pendulum links together.

- **Green Box**: The block in the green box is positioned at the end of the link. This setup is typically used for attaching additional components or for applying external forces to the system.

Similarly, Figure 3 illustrates the subsystem comprising links 2 and 3, depicted with several blocks. A rotational block is not required as the joint's rotational axis is already aligned with the world frame's y-axis.

<div class="col-sm mt-3 mt-md-0">
    {% include figure.liquid loading="eager" path="assets/img/posts/2024-05-15-Inverted-Pendulum-Day-1/03_sub_system_link2.png" class="img-fluid rounded z-depth-1" %}
</div>
<div class="caption">
    Figure 3. Detailed View of Link 2 and Link 3 Subsystem.
</div>

By meticulously arranging these blocks and understanding their specific roles, I can create a versatile simulation environment that accurately mimics real-world physical conditions. This level of detail is imperative for developing effective control strategies and for the thorough analysis of pendulum dynamics.

### C. Parameter Customization

I utilized the mask editor in Simscape to enhance subsystem customization. This tool allowed me to encapsulate subsystems, thereby facilitating easy adjustments of parameters such as mass, length, and friction through a user-friendly interface. By clicking the “Edit” button, which is highlighted in a red box in Figure 4, I was able to add new parameters to the subsystems.

<div class="col-sm mt-3 mt-md-0">
    {% include figure.liquid loading="eager" path="assets/img/posts/2024-05-15-Inverted-Pendulum-Day-1/01_overall_system_fixed.png" class="img-fluid rounded z-depth-1" %}
</div>
<div class="caption">
    Figure 4. Mask Editor Interface in Simscape.
</div>

Figure 5 displays the interface after these additions. This makes much easier to modify the parameters without altering the core model structure.

<div style="width: 60%; margin: 0 auto;">
    <div class="col-sm mt-3 mt-md-0">
        {% include figure.liquid loading="eager" path="assets/img/posts/2024-05-15-Inverted-Pendulum-Day-1/05_parameter_setting.png" class="img-fluid rounded z-depth-1" zoomable=true %}
    </div>
</div>
<div class="caption">
    Figure 5. Parameter Interface in Simscape After Masking.
</div>

### D. Simulation Results

To solve the differential equation, I utilized the **daessc** solver, which is specifically designed for Simscape DAEs (Differential-Algebraic Equations), with variable step size.

Video 1 illustrates a non-chaotic motion resulting from a specific initial angle and pendulum length. This setup reveals predictable and stable behavior, demonstrating how controlled conditions can ensure system stability.

<div style="width: 80%; margin: 0 auto;">
    <div class="col-sm mt-3 mt-md-0">
        {% include video.liquid path="assets/video/posts/2024-05-15-Inverted-Pendulum-Day-1/01_fixed_non_chaotic_motion.mp4" class="img-fluid rounded z-depth-1" controls=true muted=true %}
    </div>
</div>
<div class="caption">
    Video 1. Demonstration of Non-Chaotic Motion in the Triple Pendulum with Fixed Base Simulation.
</div>

Figure 6 provides a detailed analysis of the angular displacement for each joint of the triple pendulum system with a fixed base under non-chaotic conditions. This illustrates how each joint behaves in terms of movement and speed when the system is stable and predictable.

<div style="width: 80%; margin: 0 auto;">
    <div class="col-sm mt-3 mt-md-0">
        {% include figure.liquid loading="eager" path="assets/img/posts/2024-05-15-Inverted-Pendulum-Day-1/06_fixed_non_chaotic.png" class="img-fluid rounded z-depth-1" zoomable=true %}
    </div>
</div>
<div class="caption">
    Figure 6. Angular Displacement Analysis of Non-Chaotic Motion in the Triple Pendulum System with Fixed Base.
</div>

Conversely, Video 2 captures the onset of chaotic behavior under a slightly altered set of conditions. A small change in the initial angle leads to unpredictable and complex dynamics. This showcases the system's sensitive dependence on initial conditions—a defining characteristic of chaotic systems.

<div style="width: 80%; margin: 0 auto;">
    <div class="col-sm mt-3 mt-md-0">
        {% include video.liquid path="assets/video/posts/2024-05-15-Inverted-Pendulum-Day-1/02_fixed_chaotic_motion.mp4" class="img-fluid rounded z-depth-1" controls=true muted=true %}
    </div>
</div>
<div class="caption">
    Video 2. Demonstration of Chaotic Motion in the Triple Pendulum with Fixed Base Simulation.
</div>

Figure 7 showcases the same parameter under chaotic conditions, highlighting the significant differences when small changes in initial conditions lead to complex and unpredictable dynamics. Comparing these images helps underline the critical impact of initial setup variations on the system’s behavior, offering valuable insights for designing mechanisms to control or utilize chaotic responses effectively.

<div style="width: 80%; margin: 0 auto;">
    <div class="col-sm mt-3 mt-md-0">
        {% include figure.liquid loading="eager" path="assets/img/posts/2024-05-15-Inverted-Pendulum-Day-1/07_fixed_chaotic.png" class="img-fluid rounded z-depth-1" zoomable=true %}
    </div>
</div>
<div class="caption">
    Figure 7. Angular Displacement Analysis of Chaotic Motion in the Triple Pendulum System with Fixed Base.
</div>
<br>

# II. A Pendulum System with a Moving Base

### A. Overall System

For the pendulum with a moving frame, I kept the links unchanged but made the base movable and added visualization along the x-axis.

<div class="col-sm mt-3 mt-md-0">
    {% include figure.liquid loading="eager" path="assets/img/posts/2024-05-15-Inverted-Pendulum-Day-1/08_overall_system_moving.png" class="img-fluid rounded z-depth-1" %}
</div>
<div class="caption">
    Figure 8. Schematic of the Pendulum System with Moving Base.
</div>

### B. Subsystem - Moving Base

Figure 9 shows the moving base subsystem. Similar to the revolute joint, the translational axis of prismatic joints is the z-axis. Therefore, we need to rotate the joint's frame to align the joint's z-axis with the world frame's x-axis.

<div class="col-sm mt-3 mt-md-0">
    {% include figure.liquid loading="eager" path="assets/img/posts/2024-05-15-Inverted-Pendulum-Day-1/09_sub_system_base.png" class="img-fluid rounded z-depth-1" %}
</div>
<div class="caption">
    Figure 9. Detailed View of the Base Subsystem.
</div>

### C. Simulation Results

Video 3 depicts a non-chaotic motion resulting from a specific initial angle and pendulum length, showing predictable behavior similar to systems with a fixed base, thus illustrating how controlled conditions can enhance system stability. Figure 10 captures the parameters of the base and each joint, providing a detailed view of the setup components.

<div style="width: 80%; margin: 0 auto;">
    <div class="col-sm mt-3 mt-md-0">
        {% include video.liquid path="assets/video/posts/2024-05-15-Inverted-Pendulum-Day-1/03_moving_non_chaotic_motion.mp4" class="img-fluid rounded z-depth-1" controls=true muted=true %}
    </div>
</div>
<div class="caption">
    Video 3. Demonstration of Non-Chaotic Motion in the Triple Pendulum Simulation with Moving Base.
</div>

<div style="width: 80%; margin: 0 auto;">
    <div class="col-sm mt-3 mt-md-0">
        {% include figure.liquid loading="eager" path="assets/img/posts/2024-05-15-Inverted-Pendulum-Day-1/10_moving_non_chaotic.png" class="img-fluid rounded z-depth-1" zoomable=true %}
    </div>
</div>
<div class="caption">
    Figure 10. Angular Displacement Analysis of Non-Chaotic Motion in the Triple Pendulum System with Moving Base.
</div>

Conversely, Video 4 and Figure 11, akin to Video 2 and Figure 7, display the chaotic motion of a system with a moving base, along with the parameters of the base and each joint. The displacement of the base has been scaled up by a factor of 10 to enhance its visualization.

<div style="width: 80%; margin: 0 auto;">
    <div class="col-sm mt-3 mt-md-0">
        {% include video.liquid path="assets/video/posts/2024-05-15-Inverted-Pendulum-Day-1/04_moving_chaotic_motion.mp4" class="img-fluid rounded z-depth-1" controls=true muted=true %}
    </div>
</div>
<div class="caption">
    Video 4: Demonstration of Chaotic Motion in the Triple Pendulum Simulation with Moving Base.
</div>

<div style="width: 80%; margin: 0 auto;">
    <div class="col-sm mt-3 mt-md-0">
        {% include figure.liquid loading="eager" path="assets/img/posts/2024-05-15-Inverted-Pendulum-Day-1/11_moving_chaotic.png" class="img-fluid rounded z-depth-1" zoomable=true %}
    </div>
</div>
<div class="caption">
    Figure 11. Angular Displacement Analysis of Chaotic Motion in the Triple Pendulum System with Moving Base.
</div>

# III. Moving Forward

I chose different lengths for the pendulum primarily due to controllability concerns. Varying the lengths has helped in understanding how each variation impacts the system's ability to be controlled, thus informing the development of more effective control strategies.

In the later post, I plan to delve deeper into the relationship between the length of each link and its [controllability](https://en.wikipedia.org/wiki/Controllability). This analysis is crucial as it will inform the implementation of a controllers for the inverted pendulum. Understanding these relationships is key to developing effective control strategies that can manage the complexity and inherent challenges of chaotic systems.

It's a blend of theory and practical application, and I'm eager to see where this journey takes me in the realm of control engineering. Stay tuned for more updates as I explore the fascinating world of inverted pendulums and tackle the challenges of controlling chaotic systems!

<br>
<br>
<br>

##### You can find the Simulink model on my [GitHub repository](https://github.com/geunee20/Inverted_Pendulum).
