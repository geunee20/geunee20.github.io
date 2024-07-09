---
layout: post
title: "[Controller Design] Day 0 - Kicking Off the Controller Design Project"
date: 2024-05-14 11:31:00-0400
description: An overview of project exploring controller design techniques.
tags: Simulink Control Controller_Design
categories: Study_with_Me
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

The first phase involves building a multibody system for the multi-link mechanism. This foundational step includes designing multi-link mechanisms with different numbers of links and setting up the initial simulation environment. By understanding the basic dynamics and how the system responds under various initial conditions, we can prepare for more complex control implementations.

#### Phase 2: Achieving Stability with PID Control

Once the basic system is up and running, the focus shifts to implementing a PID controller for a multi-link mechanism with a fixed base. The application of the PID controller to the first joint (Joint 1) of the multi-link mechanism is a critical step towards achieving stable, controlled motion. Fine-tuning the PID parameters to maintain the multi-link mechanism in an upright position, despite potential disturbances, will be a test of precision and patience. This phase is pivotal as it lays the foundation for more dynamic challenges ahead.

#### Phase 3: Dynamic Adaptation with a Moving Base

Building on the stability achieved, I will then transition to experimenting with a moving base for the multi-link mechanism, which introduces a new layer of complexity. The previously static base now becomes dynamic, adding unpredictable elements to the control scenario. The challenge here is to adapt the PID controller to effectively manage these new dynamics, focusing on controlling the entire system holistically without direct actuation at Joint 1. This stage promises to be a rich ground for learning and innovation.

#### Phase 4: Advanced Control with LQR

In this penultimate phase of our project, we delve into the application of the Linear Quadratic Regulator (LQR) to control the moving base multi-link mechanism. The LQR method, a cornerstone in control theory, is celebrated for its ability to provide optimal control by minimizing a cost function that balances state errors and control effort. This powerful technique offers a robust solution for managing complex dynamic systems, making it an ideal candidate for our multi-link mechanism control challenge.

#### Phase 5: Advanced Control with Reinforcement Learning

The final frontier of this project will be the implementation of a reinforcement learning-based controller. This advanced phase will handle a moving base multi-link mechanism, still without any direct actuation at Joint 1. The objective is to enable the system to autonomously learn optimal control policies through continuous trial and error. This approach is expected to refine the system’s adaptability and optimize performance, pushing the boundaries of what can be achieved with intelligent control systems.

<br>
<br>

Each phase of this project is designed to not only build on the previous experiences but also to introduce and overcome new complexities. This systematic escalation ensures a thorough exploration and understanding of each control strategy implemented.

I invite you to stay tuned as I document this exciting journey, sharing detailed insights and results from each phase of my exploration into the dynamic realm of multi-link mechanisms. This blog will serve as a chronicle of my learning, challenges, breakthroughs, and, ultimately, the evolution of my understanding of control dynamics. Join me on this adventure as we push the limits of technology and engineering!

<br>
<br>
<br>

##### You can find the Simulink model on my [GitHub repository](https://github.com/geunee20/Controller_Design).
