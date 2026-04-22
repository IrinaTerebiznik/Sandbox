import streamlit as st
import time

st.set_page_config(page_title="Dev's Quest: Oracle", page_icon="⚔️", layout="wide")

PREP_TIMEOUT = 5 * 60  # 5 minutes

# --- SESSION STATE ---
for k, v in {'coins': 0, 'life': 3, 'prep_timer_start': None, 'force_escalation': False}.items():
    if k not in st.session_state:
        st.session_state[k] = v

def reset_game():
    st.session_state.coins = 0
    st.session_state.life = 3
    st.session_state.prep_timer_start = None
    st.session_state.force_escalation = False

# --- SIDEBAR ---
st.sidebar.header("Your Inventory")
st.sidebar.metric("Efficiency Coins", f"💰 {st.session_state.coins}")
st.sidebar.metric("Remaining Life", f"❤️ {st.session_state.life}")
if st.sidebar.button("Reset Game"):
    reset_game()

# --- TABS ---
tab_home, tab_help, tab_task, tab_pr = st.tabs([
    "🏠 Home", "🆘 Ask for Help?", "📋 Open New Task?", "✅ PR Checklist"
])

# ================================================================
# HOME
# ================================================================
with tab_home:
    st.title("🛡️ Dev's Quest: Oracle")
    st.write("Your daily toolkit for making smarter engineering decisions.")
    st.divider()

    col1, col2, col3 = st.columns(3)
    with col1:
        st.info("""
**🆘 Ask for Help?**

Evaluates whether you should stop grinding alone and escalate.
Weighs understanding, time stuck, code hygiene, and urgency.
Includes a 5-minute preparation countdown before auto-escalating.
        """)
    with col2:
        st.warning("""
**📋 Open New Task?**

Evaluates whether it's appropriate to pick up a new task while
your current one is blocked — considering clarity, blocking
situation, scope, and sprint priority.
        """)
    with col3:
        st.success("""
**✅ PR Checklist**

A comprehensive pre-push checklist to make sure your PR is
ready for review. Covers code quality, tests, git hygiene,
and PR description.
        """)

# ================================================================
# ASK FOR HELP
# ================================================================
with tab_help:
    st.title("🛡️ Dev's Quest: Are you gonna ask for help?")
    st.info("Rules: If Oracle is RED, asking for help will give you coins. Ignoring will result in life decrease.")

    with st.container():
        st.subheader("Analyze actual situation:")

        c1 = st.select_slider(
            "Do you understand task requirements and platform features involved?",
            options=[1, 2, 3, 4, 5],
            help="1: Absolutely, 5: Let's call Dora's map, 'cause I'm lost AF",
            key="help_c1"
        )
        c2 = st.select_slider(
            "How long have you been stuck with the SAME error?",
            options=[1, 2, 3, 4, 5],
            help="1: Just started, 5: Over 30 minutes",
            key="help_c2"
        )
        c3 = st.radio(
            "Have you reviewed 'Code Hygiene'?",
            ["All updated (Branches, Cache, Build)", "I need to review something"],
            key="help_c3"
        )
        val_c3 = 1 if c3 == "All updated (Branches, Cache, Build)" else 5

        c4 = st.checkbox(
            "Is this task blocking someone else or is it part of the current sprint?",
            key="help_c4"
        )
        val_c4 = 5 if c4 else 1

    # Weights: Understanding(30%), Time(30%), Hygiene(15%), Urgency(25%)
    help_score = (c1 * 0.30) + (c2 * 0.30) + (val_c3 * 0.15) + (val_c4 * 0.25)

    st.divider()

    if help_score < 2.6:
        st.session_state.prep_timer_start = None
        st.session_state.force_escalation = False
        help_mode = 'explorer'
    elif help_score >= 3.8:
        st.session_state.prep_timer_start = None
        st.session_state.force_escalation = False
        help_mode = 'escalation'
    elif st.session_state.force_escalation:
        help_mode = 'escalation'
    else:
        help_mode = 'preparation'

    if help_mode == 'explorer':
        st.success(f"Score: {help_score:.2f} | 🛠️ EXPLORER MODE: You're in control. Keep experimenting!")
        if st.button("I did it myself! (+1 Coin)"):
            st.session_state.coins += 1
            st.balloons()

    elif help_mode == 'preparation':
        if st.session_state.prep_timer_start is None:
            st.session_state.prep_timer_start = time.time()

        elapsed = time.time() - st.session_state.prep_timer_start
        remaining = PREP_TIMEOUT - elapsed

        if remaining <= 0:
            st.session_state.force_escalation = True
            st.rerun()
        else:
            mins, secs = divmod(int(remaining), 60)
            st.warning(f"Score: {help_score:.2f} | 📝 PREPARATION MODE: The path is getting murky. Document what you've tried.")
            st.info(f"⏱️ Time remaining to get unstuck: **{mins:02d}:{secs:02d}**")
            st.caption("If the timer runs out without returning to Explorer Mode, you will escalate automatically.")
            time.sleep(1)
            st.rerun()

    else:
        if st.session_state.force_escalation:
            st.error(f"Score: {help_score:.2f} | 🚨 ESCALATION MODE: Your 5 minutes are up. Ask for help now!")
        else:
            st.error(f"Score: {help_score:.2f} | 🚨 ESCALATION MODE: Ask for help now!")
        st.write("Not asking for help now will hurt your professional performance.")

        col1, col2 = st.columns(2)
        with col1:
            if st.button("I asked for help (Mission Complete)!"):
                st.session_state.coins += 2
                st.session_state.prep_timer_start = None
                st.session_state.force_escalation = False
                st.session_state.help_c1 = 1
                st.session_state.help_c2 = 1
                st.session_state.help_c3 = "All updated (Branches, Cache, Build)"
                st.session_state.help_c4 = False
                st.toast("Efficiency boost! Your team thanks you.", icon="👏")
                time.sleep(1)
                st.rerun()
        with col2:
            if st.button("I'll stay 1 more hour alone"):
                st.session_state.life -= 1
                st.toast("Careful... burnout and your managers are lurking.", icon="⚠️")
                time.sleep(1)
                st.rerun()

    if st.session_state.life <= 0:
        st.error("💀 GAME OVER: You've accumulated too much technical delay. Talk to your Lead and reset!")

# ================================================================
# OPEN NEW TASK
# ================================================================
with tab_task:
    st.title("📋 Should I Open a New Task?")
    st.info("Evaluates whether it's appropriate to pick up a new task while your current one is blocked.")

    with st.container():
        st.subheader("About the new task:")

        t1 = st.select_slider(
            "How well do you understand the new task's requirements, involved features, needed solutions, context, and can estimate the work?",
            options=[1, 2, 3, 4, 5],
            help="1: No idea, can't estimate — 5: Crystal clear, I can start right now",
            key="task_t1"
        )

        t3 = st.select_slider(
            "How long do you estimate the new task will take?",
            options=["< 30 min", "30 min – 1 hr", "1 – 2 hrs", "2 – 4 hrs", "> 4 hrs"],
            help="Shorter tasks are safer to pick up when blocked",
            key="task_t3"
        )
        task_duration_score = {"< 30 min": 5, "30 min – 1 hr": 4, "1 – 2 hrs": 3, "2 – 4 hrs": 2, "> 4 hrs": 1}[t3]

        st.subheader("About your current block:")

        t2a = st.checkbox("My current task is blocked waiting on a specific engineer's response", key="task_t2a")
        t2b = st.checkbox("I've already pinged at least one other person who could help", key="task_t2b")
        t2c = st.checkbox("I've been waiting for over an hour", key="task_t2c")
        t2d = st.checkbox("I can make meaningful progress using placeholders while waiting", key="task_t2d")

        st.subheader("Sprint context:")

        t4a = st.checkbox("My current task is blocking another sprint task", key="task_t4a")
        t4b = st.checkbox("My current task is necessary for the current sprint goal", key="task_t4b")

    # Scoring: higher = more appropriate to open the new task
    # Understanding (25%)
    score_clarity = t1 * 0.25

    # Blocking situation (35%): tried to unblock = good; can use placeholders = penalize switching
    block_raw = int(t2a) + int(t2b) + int(t2c) - (2 * int(t2d))
    block_norm = max(0, min(3, block_raw))
    score_blocking = (block_norm / 3) * 5 * 0.35

    # Task duration (25%): shorter = safer to switch
    score_duration = task_duration_score * 0.25

    # Sprint priority (15%): critical current task = penalize switching
    sprint_penalty = (2 * int(t4a)) + (2 * int(t4b))
    score_sprint = max(1, 5 - sprint_penalty) * 0.15

    task_score = score_clarity + score_blocking + score_duration + score_sprint

    st.divider()

    if task_score >= 3.5:
        st.success(f"Score: {task_score:.2f} | ✅ GO AHEAD: You're well-prepared and justified to open the new task.")
        st.write("Document your current task's state before switching so you can resume smoothly.")
    elif task_score >= 2.4:
        st.warning(f"Score: {task_score:.2f} | ⚠️ THINK TWICE: Address these gaps before opening a new task:")
        if not t2b:
            st.caption("→ Try pinging someone else who could unblock you.")
        if t2d:
            st.caption("→ You can use placeholders — no need to fully context-switch.")
        if t1 < 3:
            st.caption("→ Clarify the new task's requirements before starting it.")
        if t3 in ["> 4 hrs", "2 – 4 hrs"]:
            st.caption("→ The new task is substantial. Consider a smaller slice or spike first.")
    else:
        st.error(f"Score: {task_score:.2f} | 🚫 NOT YET: Focus on unblocking your current task first.")
        if not t2a and not t2b and not t2c:
            st.caption("→ You haven't exhausted your unblocking options yet.")
        if t2d:
            st.caption("→ You can advance with placeholders — stay on the current task.")
        if t4a or t4b:
            st.caption("→ Your current task is sprint-critical. Switching now creates risk.")

# ================================================================
# PR CHECKLIST
# ================================================================
with tab_pr:
    st.title("✅ PR / MR Checklist")
    st.info("Go through this before pushing your PR for review.")

    CHECKLIST = {
        "🧹 Code Quality": [
            "Self-reviewed all changes line by line",
            "No debug, console.log, or print statements left",
            "No commented-out code left",
            "No hardcoded secrets, tokens, or credentials",
            "Functions and variables are clearly named",
        ],
        "🧪 Testing": [
            "All existing tests pass",
            "New tests written for new functionality",
            "Happy path manually tested",
            "Edge cases and error states considered",
        ],
        "🌿 Branch & Commits": [
            "Branch is up to date with base branch (rebased or merged)",
            "No unintended files committed (.env, build artifacts, node_modules, etc.)",
            "Commit messages are clear and descriptive",
        ],
        "📝 PR Description": [
            "PR title clearly describes the change",
            "Description explains WHAT changed and WHY",
            "Screenshots or recordings attached for UI changes",
            "Breaking changes are documented",
            "Linked to the relevant ticket or task",
            "Reviewers assigned",
        ],
    }

    total = sum(len(v) for v in CHECKLIST.values())
    done = 0

    for category, items in CHECKLIST.items():
        st.subheader(category)
        for i, item in enumerate(items):
            key = f"pr_{category}_{i}"
            if st.checkbox(item, key=key):
                done += 1

    st.divider()
    st.progress(done / total, text=f"{done}/{total} items completed")

    if done == total:
        st.success("🚀 All checks passed! Your PR is ready for review.")
        st.balloons()
    elif done / total >= 0.8:
        st.warning(f"Almost there — {total - done} item(s) remaining.")
    else:
        st.info(f"{total - done} item(s) left to review.")

    if st.button("Reset Checklist"):
        for category, items in CHECKLIST.items():
            for i in range(len(items)):
                st.session_state[f"pr_{category}_{i}"] = False
        st.rerun()
