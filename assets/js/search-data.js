// get the ninja-keys element
const ninja = document.querySelector('ninja-keys');

// add the home and posts menu items
ninja.data = [{
    id: "nav-about-me",
    title: "About me",
    section: "Navigation",
    handler: () => {
      window.location.href = "/";
    },
  },{id: "nav-blog",
          title: "Blog",
          description: "",
          section: "Navigation",
          handler: () => {
            window.location.href = "/blog/";
          },
        },{id: "nav-publications",
          title: "Publications",
          description: "My publications in intelligent mechatronics and robotics.",
          section: "Navigation",
          handler: () => {
            window.location.href = "/publications/";
          },
        },{id: "nav-cv",
          title: "CV",
          description: "",
          section: "Navigation",
          handler: () => {
            window.location.href = "/cv/";
          },
        },{id: "dropdown-projects",
              title: "Projects",
              description: "",
              section: "Dropdown",
              handler: () => {
                window.location.href = "/projects/";
              },
            },{id: "dropdown-teaching",
              title: "Teaching",
              description: "",
              section: "Dropdown",
              handler: () => {
                window.location.href = "/teaching/";
              },
            },{id: "dropdown-volunteerings",
              title: "Volunteerings",
              description: "",
              section: "Dropdown",
              handler: () => {
                window.location.href = "/volunteerings/";
              },
            },{id: "post-system-modeling-leadscrew",
        
          title: "[System Modeling] Leadscrew",
        
        description: "",
        section: "Posts",
        handler: () => {
          
            window.location.href = "/blog/2024/System-Modeling-01/";
          
        },
      },{id: "post-paper-review-impedance-control-and-performance-measure-of-series-elastic-actuators",
        
          title: "[Paper Review] Impedance Control and Performance Measure of Series Elastic Actuators",
        
        description: "",
        section: "Posts",
        handler: () => {
          
            window.location.href = "/blog/2024/Paper-Review-01/";
          
        },
      },{id: "post-controller-design-day9-model-predictive-control-linearization-amp-euler-discretization",
        
          title: "[Controller Design] Day9: Model Predictive Control - Linearization &amp; Euler Discretization",
        
        description: "",
        section: "Posts",
        handler: () => {
          
            window.location.href = "/blog/2024/Controller-Design-Day-09/";
          
        },
      },{id: "post-controller-design-day8-sliding-mode-control",
        
          title: "[Controller Design] Day8: Sliding Mode Control",
        
        description: "",
        section: "Posts",
        handler: () => {
          
            window.location.href = "/blog/2024/Controller-Design-Day-08/";
          
        },
      },{id: "post-controller-design-day7-state-feedbak-control-lqr-controller-and-energy-shaping",
        
          title: "[Controller Design] Day7: State Feedbak Control - LQR Controller and Energy Shaping",
        
        description: "",
        section: "Posts",
        handler: () => {
          
            window.location.href = "/blog/2024/Controller-Design-Day-07/";
          
        },
      },{id: "post-controller-design-day-6-state-feedback-control-pole-placement-and-bang-bang",
        
          title: "[Controller Design] Day 6: State Feedback Control - Pole Placement and Bang-Bang",
        
        description: "",
        section: "Posts",
        handler: () => {
          
            window.location.href = "/blog/2024/Controller-Design-Day-06/";
          
        },
      },{id: "post-controller-design-day-5-pid-control-with-computed-torque-method",
        
          title: "[Controller Design] Day 5: PID Control with Computed Torque Method",
        
        description: "",
        section: "Posts",
        handler: () => {
          
            window.location.href = "/blog/2024/Controller-Design-Day-05/";
          
        },
      },{id: "post-controller-design-day-4-pid-control-with-gravity-compensation-method",
        
          title: "[Controller Design] Day 4: PID Control with Gravity Compensation Method",
        
        description: "",
        section: "Posts",
        handler: () => {
          
            window.location.href = "/blog/2024/Controller-Design-Day-04/";
          
        },
      },{id: "post-system-modeling-day-3-the-equations-of-motion-and-the-jacobians-of-the-triple-link-mechanism",
        
          title: "[System Modeling] Day 3: The Equations of Motion and the Jacobians of the...",
        
        description: "",
        section: "Posts",
        handler: () => {
          
            window.location.href = "/blog/2024/Controller-Design-Day-03/";
          
        },
      },{id: "post-controller-design-day-2-pid-controller",
        
          title: "[Controller Design] Day 2: PID Controller",
        
        description: "",
        section: "Posts",
        handler: () => {
          
            window.location.href = "/blog/2024/Controller-Design-Day-02/";
          
        },
      },{id: "post-controller-design-day-1-creating-the-multibody-system-using-simscape",
        
          title: "[Controller Design] Day 1: Creating the Multibody System Using Simscape",
        
        description: "",
        section: "Posts",
        handler: () => {
          
            window.location.href = "/blog/2024/Controller-Design-Day-01/";
          
        },
      },{id: "post-controller-design-day-0-kicking-off-the-controller-design-project",
        
          title: "[Controller Design] Day 0: Kicking Off the Controller Design Project",
        
        description: "",
        section: "Posts",
        handler: () => {
          
            window.location.href = "/blog/2024/Controller-Design-Day-00/";
          
        },
      },{id: "news-began-pursuing-a-master-s-degree-in-mechanical-engineering-at-georgia-tech",
          title: 'Began pursuing a Master’s degree in Mechanical Engineering at Georgia Tech!',
          description: "",
          section: "News",},{id: "news-volunteered-at-the-atlanta-science-festival",
          title: 'Volunteered at the Atlanta Science Festival!',
          description: "",
          section: "News",},{id: "news-paper-accepted-to-aim-2024-this-is-my-first-acceptance-as-a-co-author",
          title: 'Paper accepted to AIM 2024! This is my first acceptance as a co-author....',
          description: "",
          section: "News",},{id: "news-completed-my-master-s-in-mechanical-engineering-at-the-georgia-institute-of-technology",
          title: 'Completed my Master’s in Mechanical Engineering at the Georgia Institute of Technology!',
          description: "",
          section: "News",},{id: "news-volunteered-at-the-stem-atl-event-inspiring-young-minds-in-science-and-technology-across-atlanta",
          title: 'Volunteered at the STEM ATL event, inspiring young minds in science and technology...',
          description: "",
          section: "News",},{id: "news-participated-in-the-summer-science-outreach-series-at-atlanta-children-s-shelter-as-a-volunteer-bringing-hands-on-stem-activities-to-underserved-youth",
          title: 'Participated in the Summer Science Outreach Series at Atlanta Children’s Shelter as a...',
          description: "",
          section: "News",},{id: "news-started-my-first-day-at-ut-as-a-phd-student",
          title: 'Started my first day at UT as a PhD student.',
          description: "",
          section: "News",},{id: "projects-ebooth",
          title: 'eBooth',
          description: "School Event Community Application: Connecting and Boosting Student Events",
          section: "Projects",handler: () => {
              window.location.href = "/projects/4_project_ebooth/";
            },},{id: "projects-make-cornhole-playable-again",
          title: 'Make Cornhole Playable Again',
          description: "Adaptive Cornhole Assistant Tools for People with Spinal Cord Injuries",
          section: "Projects",handler: () => {
              window.location.href = "/projects/1_project_cornhole/";
            },},{id: "projects-setgo",
          title: 'SetGo',
          description: "Route Generating Application: Randomly Generates Jogging Routes Based on Distance",
          section: "Projects",handler: () => {
              window.location.href = "/projects/5_project_setGo/";
            },},{id: "projects-triple-pendulum",
          title: 'Triple Pendulum',
          description: "A Computational Tool to Mathematically Estimate the Motion of a Triple Pendulum.",
          section: "Projects",handler: () => {
              window.location.href = "/projects/3_project_triple_pendulum/";
            },},{id: "projects-robot-arm-optimization",
          title: 'Robot Arm Optimization',
          description: "Finite Element Analysis for Durability and Weight Optimization in Robot Arm Design",
          section: "Projects",handler: () => {
              window.location.href = "/projects/2_project_robot_arm/";
            },},{id: "projects-controller-design",
          title: 'Controller Design',
          description: "Exploring and implementing a spectrum of control strategies for inverted pendulum systems, from classical PID to advanced adaptive techniques",
          section: "Projects",handler: () => {
              window.location.href = "/projects/6_Controller_Design/";
            },},{id: "teachings-fall-23-39-midterm-1-review-session",
          title: 'Fall 23&amp;#39; Midterm 1 Review Session',
          description: "",
          section: "Teachings",handler: () => {
              window.location.href = "/teachings/01_23_Fa_MT1/";
            },},{id: "teachings-fall-23-39-midterm-2-review-session",
          title: 'Fall 23&amp;#39; Midterm 2 Review Session',
          description: "",
          section: "Teachings",handler: () => {
              window.location.href = "/teachings/02_23_Fa_MT2/";
            },},{id: "teachings-spring-24-39-midterm-1-review-session",
          title: 'Spring 24&amp;#39; Midterm 1 Review Session',
          description: "",
          section: "Teachings",handler: () => {
              window.location.href = "/teachings/03_24_Sp_MT1/";
            },},{id: "teachings-spring-24-39-midterm-2-review-session",
          title: 'Spring 24&amp;#39; Midterm 2 Review Session',
          description: "",
          section: "Teachings",handler: () => {
              window.location.href = "/teachings/04_24_Sp_MT2/";
            },},{
        id: 'social-email',
        title: 'email',
        section: 'Socials',
        handler: () => {
          window.open("mailto:%67%65%75%6E%65%65%32%30@%67%6D%61%69%6C.%63%6F%6D", "_blank");
        },
      },{
        id: 'social-github',
        title: 'GitHub',
        section: 'Socials',
        handler: () => {
          window.open("https://github.com/geunee20", "_blank");
        },
      },{
        id: 'social-linkedin',
        title: 'LinkedIn',
        section: 'Socials',
        handler: () => {
          window.open("https://www.linkedin.com/in/mingeun-park", "_blank");
        },
      },{
        id: 'social-scholar',
        title: 'Google Scholar',
        section: 'Socials',
        handler: () => {
          window.open("https://scholar.google.com/citations?user=BtnYdO4AAAAJ", "_blank");
        },
      },{
      id: 'light-theme',
      title: 'Change theme to light',
      description: 'Change the theme of the site to Light',
      section: 'Theme',
      handler: () => {
        setThemeSetting("light");
      },
    },
    {
      id: 'dark-theme',
      title: 'Change theme to dark',
      description: 'Change the theme of the site to Dark',
      section: 'Theme',
      handler: () => {
        setThemeSetting("dark");
      },
    },
    {
      id: 'system-theme',
      title: 'Use system default theme',
      description: 'Change the theme of the site to System Default',
      section: 'Theme',
      handler: () => {
        setThemeSetting("system");
      },
    },];
