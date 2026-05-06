---
layout: page
permalink: /teaching/
title: Teaching
description: Here is a list of courses that I am TAing/TA'ed.
nav: false
nav_order: 3
---

{% assign institutions = site.teachings | where: "teaching_type", "course" | sort: "url" | reverse | map: "institution" | uniq %}
{% for institution in institutions %}
{% unless forloop.first %}
<hr style="border: 2px solid currentColor; opacity: 0.4; margin: 2rem 0;">
{% endunless %}

### {{ institution }}

  {% assign courses = site.teachings | where: "teaching_type", "course" | sort: "url" | reverse | where: "institution", institution %}
  {% for course in courses %}
<a id="{{ course.course_id | downcase }}"></a>

**{{ course.course_id }} - {{ course.course_name }}** — *{{ course.semesters }}*
{{ course.content }}
{% unless forloop.last %}

---

{% endunless %}
  {% endfor %}
{% endfor %}
