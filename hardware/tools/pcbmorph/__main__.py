"""CLI entrypoint for `python3 -m pcbmorph`.

Commands:
  python3 -m pcbmorph model  CONFIG.json
  python3 -m pcbmorph settle CONFIG.json [--field | --no-field]
  python3 -m pcbmorph gate   CONFIG.json
  python3 -m pcbmorph run    CONFIG.json [--field | --no-field]
"""
import sys

from .common import load_config
from .model import extract_model
from .settle import run_settle
from .gate import emit_and_gate


def main(argv=None):
    argv = argv if argv is not None else sys.argv[1:]
    if len(argv) < 2:
        print(__doc__.strip(), file=sys.stderr)
        sys.exit(1)

    cmd = argv[0]
    rest = argv[1:]
    cfg_path = None
    use_field = None
    use_gpu = None
    for a in rest:
        if a == '--field':
            use_field = True
        elif a == '--no-field':
            use_field = False
        elif a == '--gpu':
            use_gpu = True
        elif a == '--no-gpu':
            use_gpu = False
        elif not a.startswith('-'):
            cfg_path = a

    if not cfg_path:
        print(__doc__.strip(), file=sys.stderr)
        sys.exit(1)

    cfg = load_config(cfg_path)

    if cmd == 'model':
        extract_model(cfg, verbose=True)
    elif cmd == 'settle':
        run_settle(cfg, use_field=use_field, use_gpu=use_gpu, verbose=True)
    elif cmd == 'gate':
        rep = emit_and_gate(cfg, verbose=True)
        if not rep['gate_passed']:
            sys.exit(2)
    elif cmd == 'run':
        extract_model(cfg, verbose=True)
        run_settle(cfg, use_field=use_field, use_gpu=use_gpu, verbose=True)
        rep = emit_and_gate(cfg, verbose=True)
        sys.stdout.flush()
        sys.stderr.flush()
        if not rep['gate_passed']:
            import os
            os._exit(2)
        import os
        os._exit(0)
    else:
        print(f"Unknown command: {cmd}\n\n" + __doc__.strip(), file=sys.stderr)
        sys.exit(1)


if __name__ == '__main__':
    main()
