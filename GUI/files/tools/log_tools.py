import datetime
import sys

def nice_print_sections(sections: dict, logfile=None, use_color=True):
    """
    sections: dict of {section_title: {label: value}}
    logfile: optional path to write a copy of output
    use_color: True to enable terminal colors, False for raw/plain text
    """

    def c(text, code):
        return f"\033[{code}m{text}\033[0m" if use_color else text

    timestamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
    output = []

    # Start header
    header = f"{c('====== MONITOR START ======', '1;36')}  [{c(timestamp, '33')}]"
    output.append(header)

    # Sections
    for title, data in sections.items():
        output.append(f"\n{c('--- ' + title + ' ---', '1;34')}")
        max_len = max(len(str(k)) for k in data.keys()) if data else 0
        for k, v in data.items():
            line = f"{str(k):{max_len}} : {v}"
            output.append(line)

    # End divider
    output.append(f"{c('====== MONITOR END ======', '1;36')}  [{c(timestamp, '33')}]")

    # Print and/or log
    for line in output:
        print(line)
    if logfile:
        try:
            with open(logfile, 'a') as f:
                f.write('\n'.join(output) + '\n')
        except Exception as e:
            print(f"[nice_print_sections] Failed to write log: {e}", file=sys.stderr)
