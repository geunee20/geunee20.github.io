---
layout: page
permalink: /volunteerings/
title: Volunteerings
description: I am dedicated to promoting equal opportunity and fostering diversity and inclusion.
nav: true
nav_order: 4
display_categories: [Diversity & Inclusion, Equal Opportunities]
horizontal: true
---

<!-- pages/volunteerings.md -->
<div class="volunteerings">
{% if site.enable_volunteering_categories and page.display_categories %}
  <!-- Display categorized volunteerings -->
  {% for category in page.display_categories %}
  <a id="{{ category }}" href=".#{{ category }}">
    <h2 class="category">{{ category }}</h2>
  </a>
  {% assign categorized_volunteerings = site.volunteerings | where: "category", category %}
  {% assign sorted_volunteerings = categorized_volunteerings | sort: "importance" %}
  <!-- Generate cards for each volunteering -->
  {% if page.horizontal %}
  <div class="container">
    <div class="row row-cols-1 row-cols-md-2">
    {% for volunteering in sorted_volunteerings %}
      {% include volunteerings_horizontal.liquid %}
    {% endfor %}
    </div>
  </div>
  {% else %}
  <div class="row row-cols-1 row-cols-md-3">
    {% for volunteering in sorted_volunteerings %}
      {% include volunteerings.liquid %}
    {% endfor %}
  </div>
  {% endif %}
  {% endfor %}

{% else %}

<!-- Display volunteerings without categories -->

{% assign sorted_volunteerings = site.volunteeringss | sort: "importance" %}

  <!-- Generate cards for each volunteering -->

{% if page.horizontal %}

  <div class="container">
    <div class="row row-cols-1 row-cols-md-2">
    {% for volunteering in sorted_volunteerings %}
      {% include volunteerings_horizontal.liquid %}
    {% endfor %}
    </div>
  </div>
  {% else %}
  <div class="row row-cols-1 row-cols-md-3">
    {% for volunteering in sorted_volunteerings %}
      {% include volunteerings.liquid %}
    {% endfor %}
  </div>
  {% endif %}
{% endif %}
</div>
