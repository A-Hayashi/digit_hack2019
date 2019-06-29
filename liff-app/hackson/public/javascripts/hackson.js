/**
* slackにメッセージを飛ばす
*/
function slack_message() {
    // 表示名前を取得
    // var name = document.getElementById('name').value;
    var name = 'chaos';
    // 表示メッセージを取得
    // var message = document.getElementById('message').value;
    var message = '地球の浄化に貢献した!';
    // チャンネル名を取得 <link|名前>でリンクを貼ることもできる 例：<http://tokyoghoul.jp/|東京喰種映画サイト>
    // var channel = document.getElementById('channel').value;
    // 送信するメッセージの作成
    var payload = JSON.stringify({'text' : message, 'username' : name});
    // var payload = JSON.stringify({'text' : message, 'username' : name, 'channel' : channel});
    // http通信オブジェクトの作成
    var request = new XMLHttpRequest ();
    // slackのwebhookが発行したURLにPOSTを送信
    request.open ('POST', 'https://hooks.slack.com/services/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');
    // headerは固定、他のheaderは送信できない
    request.setRequestHeader ('Content-Type', 'application/x-www-form-urlencoded');
    request.send (payload);
  }
  

/**
* LiFFアプリからLineBotとの連動テスト
*/
/*
window.onload = function (e) {
  // init で初期化。基本情報を取得。
  // https://developers.line.me/ja/reference/liff/#initialize-liff-app
  liff.init(function (data) {
      initializeApp(data);
  });
  // userIdの取得 wiindow.onload内無効、別のイベントでやらないと
  /*
  liff.sendMessages([
    {
      type:'text',
      text:'Hello, World!'
    }
  ])
  */
// };

function initializeApp(data) {
  console.log(data.context.userId);
}

/**
* Line個人プロフィールの取得およびメッセージ送信
*/

function getRandomInt(max) {
  return Math.floor(Math.random() * Math.floor(max));
}

function get_profile() {

  var user_name_list = ['tarou', 'pochi', 'yamada', 'yamamoto', 'takuya', 'yoshiki', 'dog', 'cat', 'hayashi', 'tebura'];
  var user_id_list = ['U000', 'U111' ,'U222' ,'U333' ,'U444' ,'U555' ,'U666' ,'U777' ,'U888' ,'U999'];
  var idx = getRandomInt(user_name_list.length);

  var user_name = user_name_list[idx];
  var user_id = user_id_list[idx];
  location.href = '/users/update?user_id=' + user_id + '&user_name=' + user_name;
  return;

    // liff.getProfile(function (data) {}の書き方うまくいかない、非同期通信だから？liff.init()は両方いけるぽい
    liff.getProfile().then(profile => {
      // var付けると、スコープは関数内にしか適用しなくて、location.hrefにうまく遷移できなくなる、グローバル宣言すると
      // うまく飛ばせるようになる
      // var user_name = profile.displayName;
      // var user_id = profile.userId;
      var user_name = profile.displayName;
      var user_id = profile.userId;
      /*
      // 外部ファイルのjavascriptの値はusers.jsにうまく渡せないため（モジュール読み込み、グローバル変数失敗、名前を蓋を開けるボタンに書き込んで、ボタン押したらデータベースに記録するやり方に変更）
      // もうちょっと考えたら、ゲットで渡したら、データベースアクセス（user/index）を開いたら、属性設定なしでもいけそう
      // liff.getProfile()内の変数をグローバルに変換したら、うまくパラメターとして飛ばせた
      // 勘違い、メソッド内でもいける、でも2回をクリックする必要がある
      document.getElementById('user_name').setAttribute(user_name, user_name);
      document.getElementById('user_id').setAttribute(user_id, user_id);
      */
      // このアドレスに飛ばす時、テーブルのCSSうまく読み込まれないため、理由はパス？テーブルのCSSをインナーで書く
      location.href = '/users/update?user_id=' + user_id + '&user_name=' + user_name;
    });      
}

/*
*　LineChatにメッセージ送信
*/
function send_message() {
  liff.sendMessages([
    {
      type:'text',
      text:'Hello, World!'
    }
  ])
}

/***************************************
           Line Things関連
****************************************/
/*
 * LineThingsライブラリ関数
*/
function line_things() {
  // service uuid
  var USER_SERVICE_UUID = '78fd82fd-cf00-4b81-92d5-73ef091fa84b';
  
  // PSDI Service UUID: Fixed value for Developer Trial
  var PSDI_SERVICE_UUID = 'e625601e-9e55-4597-a598-76018a0d293d'; // Device ID
  var PSDI_CHARACTERISTIC_UUID = '26e2b12b-85f0-4f3f-9fdd-91d114270e6e';

  liff.initPlugins(['bluetooth']).then(() => {
    liffCheckAvailablityAndDo(() => liffRequestDevice());
  }).catch(error => {
    uiStatusError(makeErrorMsg(error), false);
  });
}
