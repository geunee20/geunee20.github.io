---
layout: post
title: "[Inverted Pendulum] Day 2 - Fully Actuated Pendulum / PID Controller"
date: 2024-05-20 00:00:00-0400
description: Exploring the dynamics of a pendulum by constructing and simulating a multibody system using Simscape.
tags: Simulink Control Inverted_Pendulum
categories: Study_with_Me
disqus_comments: true
related_posts: true
thumbnail: assets/img/posts/2024-05-20-Inverted-Pendulum-Day-2/00_thumbnail.gif
images:
  compare: true
  slider: true
---

# I. A Pendulum System with a Fixed Base

### A. Overall System

<div class="col-sm mt-3 mt-md-0">
    {% include figure.liquid loading="eager" path="assets/img/posts/2024-05-20-Inverted-Pendulum-Day-2/01_fixed_overall_system.png" class="img-fluid rounded z-depth-1" %}
</div>
<div class="caption">
    Figure 1. Schematic of the Pendulum Control System with Fixed Base
</div>

### B. Subsystems

##### 1) Links

<div class="col-sm mt-3 mt-md-0">
    <swiper-container keyboard="true" navigation="true" pagination="true" pagination-clickable="true" pagination-dynamic-bullets="true" rewind="true">
        <swiper-slide>
            {% include figure.liquid loading="eager" path="assets/img/posts/2024-05-20-Inverted-Pendulum-Day-2/02_subsystem_link1.png" class="img-fluid rounded z-depth-1" %}
            <div class="caption">
                Figure 1: Detailed View of Link 1 Subsystem.
            </div>
        </swiper-slide>
        <swiper-slide>
            {% include figure.liquid loading="eager" path="assets/img/posts/2024-05-20-Inverted-Pendulum-Day-2/03_subsystem_link2.png" class="img-fluid rounded z-depth-1" %}
            <div class="caption">
                Figure 2: Detailed View of Link 2 Subsystem.
            </div>
        </swiper-slide>
        <swiper-slide>
            {% include figure.liquid loading="eager" path="assets/img/posts/2024-05-20-Inverted-Pendulum-Day-2/04_subsystem_link3.png" class="img-fluid rounded z-depth-1" %}
            <div class="caption">
                Figure 3: Detailed View of Link 3 Subsystem.
            </div>
        </swiper-slide>
    </swiper-container>
</div>

##### 2) PID Controller

### C. Simulation

##### 1) Tracking

<div class="row mt-3">
    <div class="col-sm mt-3 mt-md-0">
        {% include video.liquid path="assets/video/posts/2024-05-20-Inverted-Pendulum-Day-2/01_fixed_tracking_1.mp4" class="img-fluid rounded z-depth-1" controls=true autoplay=true %}
    </div>
    <div class="col-sm mt-3 mt-md-0">
        {% include video.liquid path="assets/video/posts/2024-05-20-Inverted-Pendulum-Day-2/02_fixed_tracking_2.mp4" class="img-fluid rounded z-depth-1" controls=true autoplay=true %}
    </div>
</div>
<div class="row mt-3">
    <div class="col-sm mt-3 mt-md-0">
        {% include video.liquid path="assets/video/posts/2024-05-20-Inverted-Pendulum-Day-2/03_fixed_tracking_3.mp4" class="img-fluid rounded z-depth-1" controls=true autoplay=true %}
    </div>
    <div class="col-sm mt-3 mt-md-0">
        {% include video.liquid path="assets/video/posts/2024-05-20-Inverted-Pendulum-Day-2/04_fixed_tracking_4.mp4" class="img-fluid rounded z-depth-1" controls=true autoplay=true %}
    </div>
</div>
<div class="caption">
    Videos 1 - 4. Demonstrating the Tracking Capabilities of the Controller Across Various Initial States in the Triple Pendulum with Fixed Base Simulation.
</div>

##### 2) Regulation

# II. A Pendulum System with a Moving Base

### A. Overall System

### B. Simulation

##### 1) Tracking

##### 2) Regulation

# III. Moving Forward

<br>
<br>
<br>

##### You can find the Simulink model on my [GitHub repository](https://github.com/geunee20/Inverted_Pendulum).
