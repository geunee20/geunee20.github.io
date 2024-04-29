---
layout: page
title: Atlanta Science Festival
description: Demonstrated building cardboard hands and exhibited a real robotic hand!
img: assets/img/volunteerings/1_atlanta_science_festival/thumbnail.png
importance: 1
category: Diversity & Inclusion
---

I was a volunteering staff at the [Atlanta Science Festival](https://research.gatech.edu/ATLScienceFestival). I prepared approximately 300 demo materials for the big weekend event. On the event day, at the DIY Robotic Hand Booth, I demonstrated how to build cardboard hands and showcased a real robotic hand for children to see and interact with.

<div class="slideshow-container">
    <div class="mySlides fade">
        <img src="/assets/img/volunteerings/1_atlanta_science_festival/img_1.jpg" class="slide-image">
    </div>
    <div class="mySlides fade">
        <img src="/assets/img/volunteerings/1_atlanta_science_festival/img_2.jpg" class="slide-image">
    </div>
    <div class="mySlides fade">
        <img src="/assets/img/volunteerings/1_atlanta_science_festival/img_3.jpg" class="slide-image">
    </div>
    <div class="mySlides fade">
        <img src="/assets/img/volunteerings/1_atlanta_science_festival/img_4.jpg" class="slide-image">
    </div>
    <div class="mySlides fade">
        <img src="/assets/img/volunteerings/1_atlanta_science_festival/img_5.jpg" class="slide-image">
    </div>
    <div class="mySlides fade">
        <img src="/assets/img/volunteerings/1_atlanta_science_festival/img_6.jpg" class="slide-image">
    </div>
    <div class="mySlides fade">
        <img src="/assets/img/volunteerings/1_atlanta_science_festival/img_7.jpg" class="slide-image">
    </div>
    <a class="prev" onclick="plusSlides(-1)">&#10094;</a>
    <a class="next" onclick="plusSlides(1)">&#10095;</a>
</div>
<div class="slider-bar-container">
  <div class="slider-bar"></div>
</div>

<script>
var slideIndex = 0;
showSlides(); 

// Next/previous controls
function plusSlides(n) {
  clearInterval(slideTimer); 
  slideIndex += n - 1;
  showSlides(); 
  slideTimer = setInterval(showSlides, 3000);
}

// Function to show a slide and set the width of the slider bar
function showSlides() {
  var i;
  var slides = document.getElementsByClassName("mySlides");
  var sliderBar = document.querySelector(".slider-bar");

  slideIndex++;
  if (slideIndex > slides.length) {slideIndex = 1}
  if (slideIndex < 1) {slideIndex = slides.length}

  for (i = 0; i < slides.length; i++) {
      slides[i].style.display = "none";  
  }
  slides[slideIndex-1].style.display = "block";  

  // Update the slider bar width
  var slideWidth = (slideIndex / slides.length) * 100;
  sliderBar.style.width = slideWidth + '%';
}

// Set the timer for automatic slideshow
var slideTimer = setInterval(showSlides, 3000);

</script>

<style>
    body {
        font-family: Arial, sans-serif;
        margin: 20px;
        color: #333;
    }
    .slideshow-container {
        width: 100%;
        position: relative;
        margin: auto;
        overflow: hidden
    }
    .mySlides {
        display: none;
        width: 100%;
        height: auto;
    }
    .slide-image {
        /* display: block; */
        width: 100%;
        height: auto%;
    }
    .prev, .next {
        cursor: pointer;
        position: absolute;
        top: 50%;
        width: auto;
        padding: 16px;
        margin-top: -22px;
        color: white;
        font-weight: bold;
        font-size: 18px;
        transition: 0.6s ease;
        border-radius: 0 3px 3px 0;
        user-select: none;
    }
    .next {
        right: 0;
        border-radius: 3px 0 0 3px;
    }
    .prev:hover, .next:hover {
        background-color: rgba(0,0,0,0.8);
    }

    .slider-bar-container {
        width: 100%;
        position: relative;
    }

    .slider-bar {
        height: 5px;
        background-color: #4CAF50;
        width: 0%; /* Initial width */
        position: absolute;
        transition: width 0.4s ease-in-out;
    }
</style>
