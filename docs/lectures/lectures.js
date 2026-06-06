/* Shared self-grading quiz engine for cargo_bot lectures.
   Usage in a lecture:
     <div class="quiz" id="quiz"></div>
     <script src="lectures.js"></script>
     <script>renderQuiz('quiz', [
        {q:'...', options:['a','b','c'], answer:1, explain:'why'}, ...
     ]);</script>
*/
function renderQuiz(containerId, quiz) {
  var root = document.getElementById(containerId);
  if (!root) return;
  var html = '<h3>Quiz &mdash; check your understanding</h3>' +
             '<p style="color:#5b6b7a;font-size:15px;margin-top:0">Pick one answer per question, then press <b>Check my answers</b>. It grades itself and explains each one.</p>';
  quiz.forEach(function (item, qi) {
    html += '<div class="q" data-answer="' + item.answer + '" id="' + containerId + '_q' + qi + '">';
    html += '<div class="qtext">' + (qi + 1) + '. ' + item.q + '</div>';
    item.options.forEach(function (opt, oi) {
      html += '<label class="opt"><input type="radio" name="' + containerId + '_q' + qi + '" value="' + oi + '">' + opt + '</label>';
    });
    html += '<div class="fb">' + (item.explain || '') + '</div>';
    html += '</div>';
  });
  html += '<button type="button">Check my answers</button>';
  html += '<div class="score"></div>';
  root.innerHTML = html;

  var btn = root.querySelector('button');
  var scoreEl = root.querySelector('.score');
  btn.addEventListener('click', function () {
    var correct = 0;
    quiz.forEach(function (item, qi) {
      var qEl = document.getElementById(containerId + '_q' + qi);
      var opts = qEl.querySelectorAll('.opt');
      opts.forEach(function (o) { o.classList.remove('correct', 'wrong'); });
      var chosen = qEl.querySelector('input:checked');
      // always highlight the right answer
      opts[item.answer].classList.add('correct');
      var fb = qEl.querySelector('.fb');
      fb.classList.add('show');
      if (chosen && parseInt(chosen.value, 10) === item.answer) {
        correct++;
        fb.className = 'fb show ok';
        fb.innerHTML = '✓ Correct. ' + (item.explain || '');
      } else {
        if (chosen) opts[parseInt(chosen.value, 10)].classList.add('wrong');
        fb.className = 'fb show no';
        fb.innerHTML = (chosen ? '✗ Not quite. ' : '— No answer. ') + (item.explain || '');
      }
    });
    var pct = Math.round((correct / quiz.length) * 100);
    var msg = correct === quiz.length ? ' 🎉 Perfect!' :
              (pct >= 70 ? ' — solid, review the reds.' : ' — re-read the lecture and try again.');
    scoreEl.textContent = 'Score: ' + correct + ' / ' + quiz.length + ' (' + pct + '%)' + msg;
    scoreEl.classList.add('show');
    scoreEl.scrollIntoView({ behavior: 'smooth', block: 'center' });
  });
}
