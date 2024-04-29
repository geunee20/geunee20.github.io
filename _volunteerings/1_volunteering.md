---
layout: page
title: Atlanta Science Festival
description: Demonstrated building cardboard hands and exhibited a real robotic hand!
img: assets/img/volunteerings/1_atlanta_science_festival/thumbnail.png
importance: 1
category: Diversity & Inclusion
---

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
    }
    .mySlides {
        display: none;
        width: 100%;
        height: auto;
    }
    .slide-image {
        width: 100%;
        height: auto;
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
    .text {
        color: #f2f2f2;
        font-size: 15px;
        padding: 8px 12px;
        position: absolute;
        bottom: 8px;
        width: 100%;
        text-align: center;
    }
    .numbertext {
        color: #f2f2f2;
        font-size: 12px;
        padding: 8px 12px;
        position: absolute;
        top: 0;
    }
</style>

I was a volunteering staff at the [Atlanta Science Festival](https://research.gatech.edu/ATLScienceFestival). I prepared approximately 300 demo materials for the big weekend event. On the event day, at the DIY Robotic Hand Booth, I demonstrated how to build cardboard hands and showcased a real robotic hand for children to see and interact with.

<div class="slideshow-container">
    <div class="mySlides fade">
        <div class="numbertext">1 / 7</div>
        <img src="/assets/img/volunteerings/1_atlanta_science_festival/img_1.jpg" class="slide-image">
    </div>
    <div class="mySlides fade">
        <div class="numbertext">2 / 7</div>
        <img src="/assets/img/volunteerings/1_atlanta_science_festival/img_2.jpg" class="slide-image">
    </div>
    <div class="mySlides fade">
        <div class="numbertext">3 / 7</div>
        <img src="/assets/img/volunteerings/1_atlanta_science_festival/img_3.jpg" class="slide-image">
    </div>
    <div class="mySlides fade">
        <div class="numbertext">4 / 7</div>
        <img src="/assets/img/volunteerings/1_atlanta_science_festival/img_4.jpg" class="slide-image">
    </div>
    <div class="mySlides fade">
        <div class="numbertext">5 / 7</div>
        <img src="/assets/img/volunteerings/1_atlanta_science_festival/img_5.jpg" class="slide-image">
    </div>
    <div class="mySlides fade">
        <div class="numbertext">6 / 7</div>
        <img src="/assets/img/volunteerings/1_atlanta_science_festival/img_6.jpg" class="slide-image">
    </div>
    <div class="mySlides fade">
        <div class="numbertext">7 / 7</div>
        <img src="/assets/img/volunteerings/1_atlanta_science_festival/img_7.jpg" class="slide-image">
    </div>
    <a class="prev" onclick="plusSlides(-1)">&#10094;</a>
    <a class="next" onclick="plusSlides(1)">&#10095;</a>
</div>

<script>
var slideIndex = 1;
showSlides(slideIndex);

function plusSlides(n) {
  showSlides(slideIndex += n);
}

function showSlides(n) {
  var i;
  var slides = document.getElementsByClassName("mySlides");
  if (n > slides.length) {slideIndex = 1}
  if (n < 1) {slideIndex = slides.length}
  for (i = 0; i < slides.length; i++) {
      slides[i].style.display = "none";  
  }
  slides[slideIndex-1].style.display = "block";  
}
</script>
