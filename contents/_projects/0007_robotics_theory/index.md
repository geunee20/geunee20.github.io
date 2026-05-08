---
layout: page
title: Algorithms for Sensor-Based Robotics Notes
description: "From SE(3) to Surgical Robotics"
date: 2026-05-08
img: ./assets/img/00_thumbnail.gif
category: Work
permalink: /projects/0007_robotics_theory/
---

{% assign asbr_tags = "Robotics_Mathematics,Robot_Kinematics,Optimization_IK,Registration_Calibration,Surgical_Robotics" | split: "," %}

{% comment %} Collect articles with ASBR category OR any of the 5 tags {% endcomment %}
{% assign all_asbr = "" | split: "" %}
{% for article in site.articles %}
{% assign in_asbr_cat = false %}
{% for cat in article.categories %}
{% if cat == "ASBR" %}
{% assign in_asbr_cat = true %}
{% endif %}
{% endfor %}
{% assign has_asbr_tag = false %}
{% for tag in article.tags %}
{% if asbr_tags contains tag %}
{% assign has_asbr_tag = true %}
{% endif %}
{% endfor %}
{% if in_asbr_cat or has_asbr_tag %}
{% assign all_asbr = all_asbr | push: article %}
{% endif %}
{% endfor %}

{% comment %} Untagged: matched via ASBR category but none of the 5 tags {% endcomment %}
{% assign untagged = "" | split: "" %}
{% for article in all_asbr %}
{% assign has_asbr_tag = false %}
{% for tag in article.tags %}
{% if asbr_tags contains tag %}
{% assign has_asbr_tag = true %}
{% endif %}
{% endfor %}
{% unless has_asbr_tag %}
{% assign untagged = untagged | push: article %}
{% endunless %}
{% endfor %}

{% assign untagged = untagged | sort: "date" | reverse %}
{% if untagged.size > 0 %}

<ul>
{% for post in untagged %}
<li><a href="{{ post.url | relative_url }}">{{ post.title }}</a></li>
{% endfor %}
</ul>
{% endif %}

{% comment %} Grouped by each tag — always show heading {% endcomment %}
{% for tag in asbr_tags %}
{% assign tag_articles = "" | split: "" %}
{% for article in all_asbr %}
{% if article.tags contains tag %}
{% assign tag_articles = tag_articles | push: article %}
{% endif %}
{% endfor %}
{% assign tag_articles = tag_articles | sort: "date" | reverse %}

<h4>{{ tag | replace: "_", " " }}</h4>
{% if tag_articles.size > 0 %}
<ul>
{% for post in tag_articles %}
<li><a href="{{ post.url | relative_url }}">{{ post.title }}</a></li>
{% endfor %}
</ul>
{% else %}
<p><em>Coming soon</em></p>
{% endif %}
{% endfor %}
