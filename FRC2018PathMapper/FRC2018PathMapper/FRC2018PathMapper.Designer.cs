namespace FRC2018PathMapper
{
  partial class FRC2018PathMapper
  {
    /// <summary>
    /// Required designer variable.
    /// </summary>
    private System.ComponentModel.IContainer components = null;

    /// <summary>
    /// Clean up any resources being used.
    /// </summary>
    /// <param name="disposing">true if managed resources should be disposed; otherwise, false.</param>
    protected override void Dispose(bool disposing)
    {
      if (disposing && (components != null))
      {
        components.Dispose();
      }
      base.Dispose(disposing);
    }

    #region Windows Form Designer generated code

    /// <summary>
    /// Required method for Designer support - do not modify
    /// the contents of this method with the code editor.
    /// </summary>
    private void InitializeComponent()
    {
      System.ComponentModel.ComponentResourceManager resources = new System.ComponentModel.ComponentResourceManager(typeof(FRC2018PathMapper));
      this.pdField = new System.Windows.Forms.PictureBox();
      this.bDraw = new System.Windows.Forms.Button();
      this.lbSteps = new System.Windows.Forms.ListBox();
      ((System.ComponentModel.ISupportInitialize)(this.pdField)).BeginInit();
      this.SuspendLayout();
      // 
      // pdField
      // 
      this.pdField.Image = global::FRC2018PathMapper.Properties.Resources.FRC2018Field;
      this.pdField.Location = new System.Drawing.Point(12, 12);
      this.pdField.Name = "pdField";
      this.pdField.Size = new System.Drawing.Size(648, 324);
      this.pdField.TabIndex = 0;
      this.pdField.TabStop = false;
      // 
      // bDraw
      // 
      this.bDraw.Location = new System.Drawing.Point(666, 313);
      this.bDraw.Name = "bDraw";
      this.bDraw.Size = new System.Drawing.Size(75, 23);
      this.bDraw.TabIndex = 1;
      this.bDraw.Text = "Draw";
      this.bDraw.UseVisualStyleBackColor = true;
      this.bDraw.Click += new System.EventHandler(this.bDraw_Click);
      // 
      // lbSteps
      // 
      this.lbSteps.FormattingEnabled = true;
      this.lbSteps.Location = new System.Drawing.Point(796, 12);
      this.lbSteps.Name = "lbSteps";
      this.lbSteps.Size = new System.Drawing.Size(188, 329);
      this.lbSteps.TabIndex = 2;
      // 
      // FRC2018PathMapper
      // 
      this.AutoScaleDimensions = new System.Drawing.SizeF(6F, 13F);
      this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
      this.ClientSize = new System.Drawing.Size(996, 352);
      this.Controls.Add(this.lbSteps);
      this.Controls.Add(this.bDraw);
      this.Controls.Add(this.pdField);
      this.Icon = ((System.Drawing.Icon)(resources.GetObject("$this.Icon")));
      this.Name = "FRC2018PathMapper";
      this.Text = "FRC 2018 Path Mapper";
      this.Paint += new System.Windows.Forms.PaintEventHandler(this.FRC2018PathMapper_Paint);
      ((System.ComponentModel.ISupportInitialize)(this.pdField)).EndInit();
      this.ResumeLayout(false);

    }

    #endregion

    private System.Windows.Forms.PictureBox pdField;
    private System.Windows.Forms.Button bDraw;
    private System.Windows.Forms.ListBox lbSteps;
  }
}

