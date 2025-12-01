---
jupytext:
    formats: md:myst
    text_representation:
        extension: .md
        format_name: myst
kernelspec:
    display_name: Python 3 (ipykernel)
    language: python
    name: python3
---

(multiverse-quiz)=
# Multiverse Quiz

This page provides a self-check quiz for the tutorial: [](multiverse).  
Source: Jupyter quiz. $ $

% NOTE: The lone `$ $` above ensures some math is rendered before the quiz,
% which fixes a known math-rendering quirk inside the quiz widget.

```{code-cell} ipython3
:tags: [remove-input]
from jupyterquiz import display_quiz

questions = [
    {
      "question": "What is the main purpose of the Multiverse Framework?",
      "type": "multiple_choice",
      "answers": [
        {"answer": "To connect multiple simulation engines together", "correct": True},
        {"answer": "To interface with different hardware components", "correct": True},
        {"answer": "To offer a unified abstraction layer for simulations", "correct": True},
        {"answer": "It supports large-scale parallel simulations on GPU tailored for reinforcement learning", "correct": False}
      ],
    },
]

import json
json_str = json.dumps(questions)
json.loads(json_str) 

display_quiz(questions)
```
