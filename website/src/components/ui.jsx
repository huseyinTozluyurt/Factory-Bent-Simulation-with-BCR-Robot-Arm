import { createContext, useContext, useEffect, useState } from 'react'
import { img } from '../data/content.js'

/* ---------- Lightbox: click any figure to see it full size ---------- */

const LightboxContext = createContext(() => {})

export function LightboxProvider({ children }) {
  const [open, setOpen] = useState(null)

  useEffect(() => {
    if (!open) return
    const onKey = (e) => e.key === 'Escape' && setOpen(null)
    window.addEventListener('keydown', onKey)
    document.body.style.overflow = 'hidden'
    return () => {
      window.removeEventListener('keydown', onKey)
      document.body.style.overflow = ''
    }
  }, [open])

  return (
    <LightboxContext.Provider value={setOpen}>
      {children}
      {open && (
        <div className="lightbox" role="dialog" aria-modal="true" onClick={() => setOpen(null)}>
          <figure onClick={(e) => e.stopPropagation()}>
            <img src={open.src} alt={open.caption} />
            {open.caption && <figcaption>{open.caption}</figcaption>}
          </figure>
          <button className="lightbox-close" aria-label="Close" onClick={() => setOpen(null)}>
            ✕
          </button>
        </div>
      )}
    </LightboxContext.Provider>
  )
}

export function Figure({ name, caption, className = '', contain = false }) {
  const openLightbox = useContext(LightboxContext)
  const src = img(name)
  return (
    <figure className={`figure ${className}`}>
      <button
        type="button"
        className={`figure-img ${contain ? 'contain' : ''}`}
        onClick={() => openLightbox({ src, caption })}
        aria-label={`Enlarge: ${caption}`}
      >
        <img src={src} alt={caption} loading="lazy" />
        <span className="zoom-hint">⤢</span>
      </button>
      {caption && <figcaption>{caption}</figcaption>}
    </figure>
  )
}

/* ---------- Section wrapper ---------- */

export function Section({ id, kicker, title, lead, children, alt = false }) {
  return (
    <section id={id} className={`section ${alt ? 'section-alt' : ''}`}>
      <div className="container">
        <header className="section-head reveal">
          {kicker && <p className="kicker">{kicker}</p>}
          <h2>{title}</h2>
          {lead && <p className="lead">{lead}</p>}
        </header>
        {children}
      </div>
    </section>
  )
}

/* ---------- Code block with a copy button ---------- */

export function Code({ children, lang = 'bash' }) {
  const [copied, setCopied] = useState(false)
  const copy = async () => {
    try {
      await navigator.clipboard.writeText(children.trim())
      setCopied(true)
      setTimeout(() => setCopied(false), 1400)
    } catch {
      /* clipboard may be blocked; ignore */
    }
  }
  return (
    <div className="code">
      <div className="code-bar">
        <span>{lang}</span>
        <button type="button" onClick={copy}>{copied ? 'Copied ✓' : 'Copy'}</button>
      </div>
      <pre>
        <code>{children.trim()}</code>
      </pre>
    </div>
  )
}

/* ---------- Before / after comparison slider ---------- */

export function Compare({ before, after, beforeLabel, afterLabel }) {
  const [pos, setPos] = useState(50)
  return (
    <div className="compare">
      <div className="compare-stage">
        <img src={img(after)} alt={afterLabel} />
        <div className="compare-top" style={{ clipPath: `inset(0 ${100 - pos}% 0 0)` }}>
          <img src={img(before)} alt={beforeLabel} />
        </div>
        <div className="compare-line" style={{ left: `${pos}%` }} />
        <span className="compare-tag left">{beforeLabel}</span>
        <span className="compare-tag right">{afterLabel}</span>
      </div>
      <input
        type="range"
        min="0"
        max="100"
        value={pos}
        onChange={(e) => setPos(Number(e.target.value))}
        aria-label="Drag to compare"
      />
    </div>
  )
}

/* ---------- Fade-in on scroll ---------- */

export function useReveal() {
  // Content is visible by default. Only once the observer is running do we
  // hide not-yet-seen blocks (html.reveal-ready), so a failed or doubled
  // effect (React StrictMode runs effects twice in dev) never hides a section.
  useEffect(() => {
    const els = document.querySelectorAll('.reveal')
    if (!('IntersectionObserver' in window)) return
    const io = new IntersectionObserver(
      (entries) =>
        entries.forEach((e) => {
          if (e.isIntersecting) {
            e.target.classList.add('shown')
            io.unobserve(e.target)
          }
        }),
      { rootMargin: '0px 0px -5% 0px' },
    )
    els.forEach((el) => {
      if (el.getBoundingClientRect().top < window.innerHeight) el.classList.add('shown')
      else io.observe(el)
    })
    document.documentElement.classList.add('reveal-ready')
    return () => {
      io.disconnect()
      document.documentElement.classList.remove('reveal-ready')
      els.forEach((el) => el.classList.add('shown'))
    }
  }, [])
}

/* ---------- Active nav item while scrolling ---------- */

export function useActiveSection(ids) {
  const [active, setActive] = useState(ids[0])
  useEffect(() => {
    const onScroll = () => {
      let current = ids[0]
      for (const id of ids) {
        const el = document.getElementById(id)
        if (el && el.getBoundingClientRect().top < 140) current = id
      }
      setActive(current)
    }
    onScroll()
    window.addEventListener('scroll', onScroll, { passive: true })
    return () => window.removeEventListener('scroll', onScroll)
  }, [ids])
  return active
}
