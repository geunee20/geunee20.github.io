---
layout: page_cv
permalink: /cv/
title: CV
nav: true
nav_order: 5
cv_pdf: /assets/pdf/240924_Min_Geun_Park.pdf
description:
toc:
  sidebar: false
_styles: |
  .cv-pdf-wrap {
    width: 100%;
    height: clamp(540px, 88vh, 1400px);
    overflow: hidden;
    border: 1px solid var(--global-divider-color);
    border-radius: 0.5rem;
  }

  .cv-pdf-frame {
    width: 100%;
    height: 100%;
    border: 0;
  }

  .cv-fallback-note {
    margin-top: 0.75rem;
  }
---

<div class="cv-pdf-wrap">
  <iframe
    src="{{ page.cv_pdf | relative_url }}#navpanes=0&zoom=page-width"
    class="cv-pdf-frame"
    title="CV PDF"
  ></iframe>
</div>

<p class="cv-fallback-note">
  If the PDF does not load,
  <a href="{{ '/web_cv/' | relative_url }}">click here to view the web version.</a>
</p>
