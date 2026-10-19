# Rotary Inverted Pendulum

[![Watch the build video](assets/youtube-thumbnail-pendulum-build.jpg)](https://www.youtube.com/watch?v=rKChjuuR7K8)

📖 **[Read the documentation →](https://ferrolho.github.io/rotary-inverted-pendulum/)** — guided build and training path, with an interactive 3D demo that runs the deployed network in your browser.

A DIY rotary inverted pendulum you can print, solder, and train at home — for about **£20** in parts. It's an open, hackable take on the rigs you'd usually buy from a lab-equipment vendor (Quanser's [QUBE Servo 2](https://www.quanser.com/products/qube-servo-2) lists at around £4,500). The pendulum balances itself with a reinforcement-learning policy trained in simulation, fine-tuned on the real hardware, and distilled to run standalone on an Arduino Nano.

## What's in this repo

The layout follows the pipeline: measure the rig, train a policy, shrink it, flash it.

| Directory                 | Contents                                                                                                      |
| ------------------------- | ------------------------------------------------------------------------------------------------------------- |
| [`policy/`](policy)       | The RL pipeline — MuJoCo sim env, SAC training, system identification, real-rig bridge, distillation and weight export |
| [`firmware/`](firmware)   | Arduino sketches — the standalone RL controller, the low-level server used for fine-tuning, and the bring-up tests |
| [`model/`](model)         | The URDF and the 3D-printable STLs it references (single source of truth for pendulum geometry)                |
| [`hardware/`](hardware)   | Wiring diagrams and component photos                                                                          |
| [`website/`](website)     | All documentation (`src/content/docs/`) plus the site itself — Astro Starlight and the in-browser MuJoCo demo  |

## Where to start

- **Build one** — [bill of materials](https://ferrolho.github.io/rotary-inverted-pendulum/build/bom/) → [print](https://ferrolho.github.io/rotary-inverted-pendulum/build/printing/) → [wire](https://ferrolho.github.io/rotary-inverted-pendulum/build/electronics/) → [first power-on](https://ferrolho.github.io/rotary-inverted-pendulum/build/first-power-on/)
- **Train a policy** — the [end-to-end pipeline](https://ferrolho.github.io/rotary-inverted-pendulum/train/pipeline/), step 0 through step 4
- **Understand the RL stack** — [the transition contract](https://ferrolho.github.io/rotary-inverted-pendulum/reference/transitions/), [domain randomization](https://ferrolho.github.io/rotary-inverted-pendulum/reference/domain-randomization/), [transport delay](https://ferrolho.github.io/rotary-inverted-pendulum/reference/transport-delay/)

## Prefer to buy rather than build?

DIY kits run [$100–$200 on AliExpress](https://www.aliexpress.com/w/wholesale-rotary-inverted-pendulum.html); the Quanser QUBE Servo 2 mentioned above is around £4,500.

## Related work

- [Desktop Inverted Pendulum, build-its-inprogress](https://build-its-inprogress.blogspot.com/2016/08/desktop-inverted-pendulum-part-2-control.html) ([full series](https://build-its-inprogress.blogspot.com/search/label/Pendulum))
- [Furuta pendulum, dagor.dev](https://www.dagor.dev/blog/furuta-pendulum)
- [The Rotary Control Lab — Quanser brochure (PDF)](https://tecsolutions.us/sites/default/files/quanser/The%20Rotary%20Control%20Lab%20Brochure_4.pdf)
- [Survey paper, *Trans. Inst. Meas. Control*](https://journals.sagepub.com/doi/full/10.1177/00202940211035406)
- Video builds: [[1]](https://www.youtube.com/watch?v=2koXcs0IhOc), [[2]](https://www.youtube.com/watch?v=bY4t6yfBA24), [[3]](https://www.youtube.com/watch?v=VVQ-PGfJMuA)

## Acknowledgments

I would like to thank the following people for their contributions to this project:
- [Joe](https://github.com/spookycouch) for suggesting I try reinforcement learning with Stable Baselines 3, which kicked off the learned-control parts of this project.
- [Mykha](https://github.com/Mika412) for early discussions about this project over a beer in the park.
- [André](https://github.com/Esser50K), [Rafael](https://github.com/rkourdis), and [Vlad](https://github.com/VladimirIvan) for technical discussions, feedback, and support.
- [Vivek](https://github.com/svrkrishnavivek) for his invaluable help and feedback on the electronics of the system.
- [心诺 (Xinnuo)](https://github.com/XinnuoXu) for her company and support while working on this project.

Finally, I would like to thank the open-source community in general for providing the tools and resources that have also helped make this project possible.
