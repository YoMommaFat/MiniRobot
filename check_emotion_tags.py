"""How well does a local model keep MiniRobot's emotion tags?

Runs on the PC next to Ollama, no ROS needed, only the Python standard library:

    python3 check_emotion_tags.py --prompt robot_prompt.json
    python3 check_emotion_tags.py --prompt robot_prompt.json --model qwen3-vl:4b-instruct --runs 3

It sends Hungarian test lines to the model with MiniRobot's system prompt, plus three
made-up results of the nod_towards tool and one longer conversation, and checks how each
reply starts: a valid tag, an unknown tag, a lowercase tag, or none. All replies are
printed so you can judge the Hungarian yourself; they are also saved to --out.
"""
import argparse
import json
import re
import statistics
import time
import urllib.request

VALID = ["READY", "CONFUSED", "SUCCESS", "SORRY", "CURIOSITY", "OBSERVING", "AFFECTION",
         "WINK", "ALERT"]

# What the user says, and the tags that fit (loose: any of them counts as a good choice).
LINES = [
    ("Szia! Hogy vagy ma?", {"READY", "AFFECTION"}),
    ("Hogy hívnak?", {"READY", "AFFECTION"}),
    ("Köszönöm szépen a segítséget, nagyon ügyes vagy!", {"AFFECTION", "SUCCESS"}),
    ("Mondj egy viccet!", {"WINK"}),
    ("izé... hogy is mondjam... a hogyishívják", {"CONFUSED", "CURIOSITY"}),
    ("Mennyi most a hőmérséklet a Marson?", {"SORRY", "CONFUSED"}),
    ("Mennyi tizenkettő meg harminc?", {"SUCCESS", "READY"}),
    ("Vigyázz, mindjárt leesel az asztalról!", {"ALERT"}),
    ("Elromlott a bal kereked, nem forog.", {"ALERT", "SORRY"}),
    ("Szerinted mit csináljak a hétvégén?", {"CURIOSITY", "READY"}),
    ("Kérdezz tőlem valamit!", {"CURIOSITY"}),
    ("Nézd meg, ki jött be az ajtón!", {"OBSERVING", "CURIOSITY", "SORRY"}),
]

# Lines added to the system prompt if robot_prompt.json does not mention the tool yet.
TOOL_LINES = [
    "Ha azt kérik, hogy menj oda valamihez vagy közelíts hozzá, hívd a nod_towards eszközt a tárgy rövid angol nevével (például 'red ball').",
    "Az eszköz eredménye megmondja, melyik állapotkóddal kezdd a válaszod: azt használd.",
]

TOOL = {"type": "function", "function": {
    "name": "nod_towards",
    "description": "Turn towards a visible object and drive a few short steps to it.",
    "parameters": {"type": "object", "required": ["target"], "properties": {
        "target": {"type": "string", "description": "short English name, e.g. 'red ball'"}}}}}

# (user line, what the tool returned, the tag the tool asked for)
TOOL_CASES = [
    ("Menj oda a piros labdához!",
     "Kész: 15 fokot fordultam jobbra, és 3 lépésben 15 cm-t mentem a red ball felé. "
     "A válaszod ezzel kezdődjön: [SUCCESS]", "SUCCESS"),
    ("Menj oda a kék bögréhez!",
     "Nem látom a blue mug-ot magam előtt, nem mozdultam. A válaszod ezzel kezdődjön: [SORRY]",
     "SORRY"),
    ("Közelíts a sárga dobozhoz!",
     "Nem indultam el: a távolságmérők nem küldenek adatot. "
     "A válaszod ezzel kezdődjön: [ALERT]", "ALERT"),
]

# One longer conversation: does the model keep the tags turn after turn?
CHAT = ["Szia!", "Mi a kedvenc színed?", "És miért pont az?", "Mondj egy viccet!",
        "Ez nem volt vicces.", "Jó, akkor köszönöm, szia!"]

TAG_AT_START = re.compile(r"^\s*['\"]?\[([A-Za-z_]+)\]")
ANY_TAG = re.compile(r"\[([A-Za-z_]+)\]")
CJK = re.compile(r"[぀-ヿ㐀-鿿가-힯]")
EMOJI = re.compile(r"[\U0001F300-\U0001FAFF☀-➿]")
HU_LETTERS = re.compile(r"[áéíóöőúüű]", re.IGNORECASE)


def classify(text):
    """How the reply's tag looks: ok, late, unknown, lowercase or missing."""
    m = TAG_AT_START.match(text)
    if m:
        tag = m.group(1)
        if tag in VALID:
            return "ok", tag
        if tag.upper() in VALID:
            return "lowercase", tag      # tts_node only strips [A-Z_]: it would be read out
        return "unknown", tag
    later = [t for t in ANY_TAG.findall(text) if t in VALID]
    if later:
        return "late", later[0]          # the face changes only when the tag arrives
    return "missing", None


def flags(text):
    out = []
    if CJK.search(text):
        out.append("chinese/japanese characters")
    if EMOJI.search(text):
        out.append("emoji")
    if re.search(r"\*\*|^\s*[-*#]\s", text, re.MULTILINE):
        out.append("markdown")
    body = ANY_TAG.sub("", text)
    if len(body) > 40 and not HU_LETTERS.search(body):
        out.append("no Hungarian accents (English?)")
    if len(body) > 400:
        out.append(f"long ({len(body)} chars)")
    return out


def chat(url, model, messages, tools=None, temperature=None):
    body = {"model": model, "messages": messages, "stream": False}
    if tools:
        body["tools"] = tools
    if temperature is not None:
        body["options"] = {"temperature": temperature}
    req = urllib.request.Request(url.rstrip("/") + "/api/chat", data=json.dumps(body).encode(),
                                 headers={"Content-Type": "application/json"})
    start = time.time()
    with urllib.request.urlopen(req, timeout=300) as r:
        reply = json.loads(r.read())["message"]
    return reply, time.time() - start


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--prompt", default="robot_prompt.json", help="MiniRobot's robot_prompt.json")
    ap.add_argument("--model", default="qwen3-vl:4b-instruct")
    ap.add_argument("--url", default="http://localhost:11434")
    ap.add_argument("--runs", type=int, default=3, help="how many times each line is sent")
    ap.add_argument("--temperature", type=float, default=None, help="default: the model's own")
    ap.add_argument("--out", default="emotion_check.jsonl")
    a = ap.parse_args()

    with open(a.prompt, encoding="utf-8") as f:
        raw = json.load(f)["system_prompt"]
    prompt = "\n".join(raw) if isinstance(raw, list) else str(raw)
    tool_prompt = prompt if "nod_towards" in prompt else prompt + "\n" + "\n".join(TOOL_LINES)

    results = []
    out = open(a.out, "w", encoding="utf-8")

    def record(kind, user, text, seconds, fits=None, wanted=None):
        status, tag = classify(text)
        r = {"kind": kind, "user": user, "reply": text, "status": status, "tag": tag,
             "fits": (tag in fits) if fits and status == "ok" else None,
             "as_asked": (tag == wanted) if wanted else None,
             "flags": flags(text), "seconds": round(seconds, 1)}
        results.append(r)
        out.write(json.dumps(r, ensure_ascii=False) + "\n")
        out.flush()
        mark = {"ok": "OK ", "late": "LATE", "unknown": "UNK", "lowercase": "LOW",
                "missing": "MISS"}[status]
        extra = f"  !! {', '.join(r['flags'])}" if r["flags"] else ""
        print(f"[{mark}] {seconds:5.1f}s  {user}\n        -> {text.strip()[:300]}{extra}")

    print(f"Model {a.model} at {a.url}, {a.runs} run(s) per line\n")
    print("== Single lines")
    for user, fits in LINES:
        for _ in range(a.runs):
            msg, s = chat(a.url, a.model, [{"role": "system", "content": prompt},
                                           {"role": "user", "content": user}],
                          temperature=a.temperature)
            record("line", user, msg.get("content") or "", s, fits=fits)

    print("\n== After a nod_towards result (the reply should start with the tag it asks for)")
    for user, result, wanted in TOOL_CASES:
        for _ in range(a.runs):
            first, s1 = chat(a.url, a.model, [{"role": "system", "content": tool_prompt},
                                              {"role": "user", "content": user}],
                             tools=[TOOL], temperature=a.temperature)
            called = bool(first.get("tool_calls"))
            msgs = [{"role": "system", "content": tool_prompt}, {"role": "user", "content": user},
                    {"role": "assistant", "content": "", "tool_calls": first.get("tool_calls") or [
                        {"function": {"name": "nod_towards", "arguments": {"target": "object"}}}]},
                    {"role": "tool", "content": result, "tool_name": "nod_towards"}]
            msg, s2 = chat(a.url, a.model, msgs, tools=[TOOL], temperature=a.temperature)
            text = msg.get("content") or ""
            if msg.get("tool_calls"):
                text += f"  <called the tool again: {msg['tool_calls']}>"
            label = user + ("" if called else "  (model did NOT call the tool itself)")
            record("tool", label, text, s1 + s2, wanted=wanted)

    print("\n== One longer conversation")
    msgs = [{"role": "system", "content": prompt}]
    for user in CHAT:
        msgs.append({"role": "user", "content": user})
        msg, s = chat(a.url, a.model, msgs, temperature=a.temperature)
        text = msg.get("content") or ""
        msgs.append({"role": "assistant", "content": text})
        record("chat", user, text, s)
    out.close()

    print("\n== Summary")
    for kind, name in (("line", "single lines"), ("tool", "after a tool result"),
                       ("chat", "longer conversation")):
        rs = [r for r in results if r["kind"] == kind]
        if not rs:
            continue
        counts = {k: sum(r["status"] == k for r in rs) for k in
                  ("ok", "late", "unknown", "lowercase", "missing")}
        line = f"{name:22} {counts['ok']}/{len(rs)} start with a valid tag"
        bad = ", ".join(f"{v} {k}" for k, v in counts.items() if k != "ok" and v)
        if bad:
            line += f" ({bad})"
        if kind == "line":
            fit = [r for r in rs if r["fits"] is not None]
            line += f"; {sum(r['fits'] for r in fit)}/{len(fit)} of those tags fit the line"
        if kind == "tool":
            line += f"; {sum(bool(r['as_asked']) for r in rs)}/{len(rs)} used the tag it was given"
        print(line)
    flagged = [r for r in results if r["flags"]]
    print(f"{'flagged replies':22} {len(flagged)}/{len(results)} (see '!!' above)")
    print(f"{'reply time':22} median {statistics.median(r['seconds'] for r in results):.1f} s")
    unknown = sorted({r["tag"] for r in results if r["status"] in ("unknown", "lowercase")})
    if unknown:
        print(f"{'tags it made up':22} {', '.join(unknown)}")
    print(f"\nAll replies saved to {a.out}")


if __name__ == "__main__":
    main()
