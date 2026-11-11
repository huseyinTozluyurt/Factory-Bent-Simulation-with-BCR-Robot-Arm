# Factory Bent project website

A React + Vite site that explains the Factory Bent ROS 2 project: overview,
architecture, worlds, the vision pipeline, pick and place, the product
generator, the pipeline factory, the vision lab, the evolution timeline and
how to run it.

The site reads its images and PDF reports straight from the project's
`docs/` folder (`publicDir: '../docs'` in `vite.config.js`), so keep this
folder at `factory_bent_project/website/`. All text lives in
`src/data/content.js`.

## Run it

Needs Node.js 20.19 or newer.

```bash
cd ~/ros2_ws/src/factory_bent_project/website
npm install
npm run dev        # opens http://localhost:5173
```

## Build a static copy

```bash
npm run build      # writes dist/ (relative paths, works from any folder or GitHub Pages)
npm run preview    # serves dist/ on http://localhost:4173
```

## Layout

```
website/
├── index.html
├── vite.config.js           base './', publicDir '../docs'
└── src/
    ├── App.jsx              all page sections
    ├── data/content.js      text, tables, timeline and image names
    ├── components/ui.jsx    figure + lightbox, code block, compare slider, section
    ├── components/diagrams.jsx  architecture and pipeline SVG diagrams, cycle stepper
    └── styles.css
```

To add a screenshot, put it in `docs/images/` and reference it by file name
with `<Figure name="my_image.png" caption="..." />`.
