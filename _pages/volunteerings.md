---
layout: page
permalink: /volunteerings/
title: Volunteerings
description: I am dedicated to promoting equal opportunity and fostering diversity and inclusion.
nav: false
nav_order: 4
---

{% assign entries = site.volunteerings | sort: "url" | reverse %}
{% for entry in entries %}
<a id="{{ entry.anchor }}"></a>
{{ entry.content }}
{% unless forloop.last %}

---

{% endunless %}
{% endfor %}
